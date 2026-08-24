import time
import board
import busio
import digitalio
import adafruit_rfm9x


def now_us():
    return time.monotonic_ns() // 1000


# ============================================================
# ID / RADIO
# ============================================================

ID_TRAM_SU = 0x01
ID_TRAM_DU = 0x02
ID_TRAM_RBS = 0x03

TAN_SO_LORA = 433.0

TYPE_VOICE = 0x01
TYPE_SESSION_START = 0x02
TYPE_READY = 0x03
TYPE_AUDIO_END_SU = 0x04
TYPE_FEC = 0x05

TYPE_MASK = 0x0F
FLAG_LAST = 0x10
COUNT_SHIFT = 5

SIZE_SESSION = 12
SIZE_VOICE_PACKET = 176
SIZE_FEC_PACKET = 184
SIZE_CONTROL_SU = 5

VOICE_LENGTH = 168

TYPE_RELAY = 0x10
TYPE_RELAY_END = 0x11

# Control DU -> rBS -> SU de do PTT_RELEASE -> DU_PLAY.
TYPE_PLAY_STARTED = 0x12
SIZE_PLAY_STARTED = 12

# Sau END #1, rBS mo RX window cho DU bao PLAY.
# Playback DU da bat dau ngay tu END #1; window nay KHONG chen delay vao loa.
PLAY_REPORT_WINDOW = 0.120
PLAY_REPORT_FORWARD_GUARD = 0.008

# Giữ các guard đã chứng minh ổn định.
PHASE2_RX_GUARD = 0.008
READY_GUARD = 0.010

# Explicit AUDIO_END là đường bình thường.
# Fallback để dài hơn ARQ retry, không ảnh hưởng latency bình thường.
END_OF_AUDIO_IDLE = 1.500


# ============================================================
# PARSER SU
# ============================================================

def parse_packet_su(packet_bytes):

    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)

    if len(p) not in (
        SIZE_SESSION,
        SIZE_VOICE_PACKET,
        SIZE_FEC_PACKET,
        SIZE_CONTROL_SU,
    ):
        return None

    dst = p[0]
    src = p[1]
    packet_type = p[2] & TYPE_MASK

    if dst != ID_TRAM_DU or src != ID_TRAM_SU:
        return None

    # --------------------------------------------------------
    # AUDIO_END 5B
    # --------------------------------------------------------
    if len(p) == SIZE_CONTROL_SU:
        if packet_type == TYPE_AUDIO_END_SU:
            return {
                "kind": "audio_end",
                "raw": p,
            }

        return None

    # --------------------------------------------------------
    # SESSION
    # --------------------------------------------------------
    if packet_type == TYPE_SESSION_START:
        if len(p) != SIZE_SESSION or p[3] != 8:
            return None

        session_id = int.from_bytes(
            p[4:12],
            byteorder="big",
            signed=False,
        )

        if session_id == 0:
            return None

        return {
            "kind": "session",
            "raw": p,
            "session_id": session_id,
        }

    # --------------------------------------------------------
    # VOICE 176B
    #
    # byte2:
    #   bits7..5 frame_count-1
    #   bit4     LAST
    #   bits3..0 TYPE
    #
    # byte3 = 168
    # --------------------------------------------------------
    if packet_type == TYPE_VOICE:
        if len(p) != SIZE_VOICE_PACKET:
            return None

        if p[3] != VOICE_LENGTH:
            return None

        frames = ((p[2] >> COUNT_SHIFT) & 0x07) + 1
        last_audio = (p[2] & FLAG_LAST) != 0

        seq = int.from_bytes(
            p[4:8],
            byteorder="big",
            signed=False,
        )

        if seq & 0x80000000:
            return None

        return {
            "kind": "voice",
            "raw": p,
            "seq": seq,
            "frames": frames,
            "last_audio": last_audio,
            "ack_kind": TYPE_VOICE,
            "ack_seq": seq,
        }

    # --------------------------------------------------------
    # FEC 176B
    #
    # byte2:
    #   bits7..5 data_count-1
    #   bit4     GROUP_HAS_LAST
    #   bits3..0 TYPE_FEC
    #
    # byte3:
    #   0 hoặc final_frame_count 1..8
    #
    # byte4..7 group_start_seq
    # --------------------------------------------------------
    if packet_type == TYPE_FEC:
        if len(p) != SIZE_FEC_PACKET:
            return None

        data_count = ((p[2] >> COUNT_SHIFT) & 0x07) + 1
        has_last = (p[2] & FLAG_LAST) != 0
        final_frames = p[3]

        if has_last:
            if final_frames < 1 or final_frames > 8:
                return None
        else:
            if final_frames != 0:
                return None

        group_start = int.from_bytes(
            p[4:8],
            byteorder="big",
            signed=False,
        )

        if group_start & 0x80000000:
            return None

        return {
            "kind": "fec",
            "raw": p,
            "group_start": group_start,
            "data_count": data_count,
            "has_last": has_last,
            "final_frames": final_frames,
            "ack_kind": TYPE_FEC,
            "ack_seq": group_start,
        }

    return None


# ============================================================
# RELAY / CONTROL
# ============================================================

def gui_relay_wrapper(rfm9x, packet_goc):

    packet_goc = bytes(packet_goc)

    if len(packet_goc) not in (
        SIZE_SESSION,
        SIZE_VOICE_PACKET,
        SIZE_FEC_PACKET,
    ):
        raise ValueError(
            f"Do dai packet goc khong hop le: {len(packet_goc)}"
        )

    # RadioHead 4B outer header.
    # VOICE 176B -> 180B physical relay.
    # FEC   184B -> 188B physical relay.
    rfm9x.send(
        packet_goc,
        destination=ID_TRAM_DU,
        node=ID_TRAM_RBS,
        identifier=TYPE_RELAY,
        flags=len(packet_goc),
    )


def gui_ready_ack(
    rfm9x,
    ack_kind,
    ack_seq,
):
    # 4B RadioHead header + 4B payload seq = 8B.
    rfm9x.send(
        int(ack_seq).to_bytes(
            4,
            byteorder="big",
            signed=False,
        ),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_READY,
        flags=ack_kind,
    )

    print(
        f"[rBS CTRL] ACK -> SU | "
        f"KIND=0x{ack_kind:02X} | SEQ={ack_seq}"
    )


def parse_play_started_du(packet_bytes, expected_session_id):

    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)

    # DU tu dong tao RadioHead-compatible header 4B + session 8B.
    if len(p) != SIZE_PLAY_STARTED:
        return None

    if (
        p[0] != ID_TRAM_RBS
        or p[1] != ID_TRAM_DU
        or p[2] != TYPE_PLAY_STARTED
    ):
        return None

    session_id = int.from_bytes(
        p[4:12],
        byteorder="big",
        signed=False,
    )

    if session_id == 0:
        return None

    if (
        expected_session_id is not None
        and session_id != expected_session_id
    ):
        return None

    return session_id


def gui_play_started_cho_su(rfm9x, session_id):

    rfm9x.send(
        int(session_id).to_bytes(
            8,
            byteorder="big",
            signed=False,
        ),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_PLAY_STARTED,
        flags=0,
    )

    print(
        f"[rBS CTRL] PLAY_STARTED -> SU | "
        f"SESSION={session_id:016X}"
    )


def gui_end_audio_cho_du(rfm9x, expected_session_id=None):
    """
    END #1 -> DU bat dau PLAY.
    rBS lap tuc mo RX de nhan PLAY_STARTED cua DU, forward ve SU.
    Sau do moi gui END #2/#3 lam redundancy nhu baseline cu.

    Nhu vay reverse report khong can cho het 3 END, nen phep do E2E
    chi bi cong them airtime cua 2 packet report nho (~20.6 ms).
    """

    # END #1 - day la END lam DU mo play gate binh thuong.
    rfm9x.send(
        b"\x00",
        destination=ID_TRAM_DU,
        node=ID_TRAM_RBS,
        identifier=TYPE_RELAY_END,
        flags=0,
    )

    print("[rBS CTRL] END_AUDIO #1 -> DU | MO CUA SO PLAY_REPORT")

    got_play_report = False
    deadline = time.monotonic() + PLAY_REPORT_WINDOW

    while time.monotonic() < deadline:
        p = rfm9x.receive(
            timeout=0.005,
            with_header=True,
        )

        if p is None:
            continue

        sid = parse_play_started_du(
            p,
            expected_session_id,
        )

        if sid is None:
            continue

        print(
            f"[rBS RX CTRL] PLAY_STARTED FROM DU | "
            f"SESSION={sid:016X}"
        )

        # SU cung nghe thay packet DU->rBS. Cho SU doc/xả packet do
        # va re-arm RX continuous truoc khi rBS forward report.
        time.sleep(PLAY_REPORT_FORWARD_GUARD)

        print(
            f"[TIME][rBS] PLAY_REPORT_FORWARD_GUARD = "
            f"{PLAY_REPORT_FORWARD_GUARD * 1000:.1f} ms"
        )

        gui_play_started_cho_su(
            rfm9x,
            sid,
        )

        got_play_report = True
        break

    if not got_play_report:
        print(
            "[rBS CTRL] PLAY_REPORT TIMEOUT -> SU se hien E2E ---"
        )

    # Giu redundancy END x3: gui not END #2 va #3.
    for lan in range(2):
        rfm9x.send(
            b"\x00",
            destination=ID_TRAM_DU,
            node=ID_TRAM_RBS,
            identifier=TYPE_RELAY_END,
            flags=0,
        )

        if lan == 0:
            time.sleep(0.020)

    print("[rBS CTRL] END_AUDIO #2/#3 -> DU")


# ============================================================
# RELAY 1 PACKET DATA (VOICE/FEC)
# ============================================================

def relay_one_data_packet(
    rfm9x,
    info,
    goi_session,
    session_da_gui,
    t_rx_us,
):

    t_start = now_us()

    print()
    print("----------------------------------------------------")
    print(
        f"[rBS] PHASE 2 BAT DAU | RELAY 1 {info['kind'].upper()}"
    )
    print("----------------------------------------------------")

    if t_rx_us is not None:
        print(
            f"[TIME][rBS] RX_LAST -> PHASE2_START = "
            f"{t_start - t_rx_us} us"
        )

    # Burst đầu có SESSION relay tạo khoảng đệm tự nhiên.
    if session_da_gui:
        t0 = now_us()
        time.sleep(PHASE2_RX_GUARD)
        t1 = now_us()

        print(
            f"[TIME][rBS] PHASE2_RX_GUARD = "
            f"{(t1 - t0) / 1000:.3f} ms"
        )

    if (
        not session_da_gui
        and goi_session is not None
    ):
        session_id = int.from_bytes(
            goi_session[4:12],
            byteorder="big",
            signed=False,
        )

        for lan in range(2):
            gui_relay_wrapper(
                rfm9x,
                goi_session,
            )

            if lan == 0:
                time.sleep(0.005)

        print(
            f"[rBS TX RELAY] SESSION_START | "
            f"SESSION={session_id:016X}"
        )

        session_da_gui = True

    t_tx0 = now_us()

    gui_relay_wrapper(
        rfm9x,
        info["raw"],
    )

    t_tx1 = now_us()

    if info["kind"] == "voice":
        print(
            f"[rBS TX RELAY] VOICE | "
            f"SEQ={info['seq']} | "
            f"FRAME={info['frames']} | "
            f"LAST={1 if info['last_audio'] else 0}"
        )
    else:
        print(
            f"[rBS TX RELAY] FEC | "
            f"GROUP_START={info['group_start']} | "
            f"DATA={info['data_count']} | "
            f"HAS_LAST={1 if info['has_last'] else 0}"
        )

    print(
        f"[TIME][rBS TX] "
        f"DUR={t_tx1 - t_tx0} us "
        f"({(t_tx1 - t_tx0) / 1000:.3f} ms)"
    )

    # Cho SU xả packet relay 180B nghe ké và quay lại RX.
    tg0 = now_us()
    time.sleep(READY_GUARD)
    tg1 = now_us()

    print(
        f"[TIME][rBS] READY_GUARD = "
        f"{(tg1 - tg0) / 1000:.3f} ms"
    )

    ta0 = now_us()

    gui_ready_ack(
        rfm9x,
        info["ack_kind"],
        info["ack_seq"],
    )

    ta1 = now_us()

    print(
        f"[TIME][rBS] ACK_TX_DUR = "
        f"{ta1 - ta0} us "
        f"({(ta1 - ta0) / 1000:.3f} ms)"
    )

    print("[rBS] PHASE 2 KET THUC")
    print("[rBS] PHASE 1 -> QUAY LAI RX SU")
    print("----------------------------------------------------")
    print()

    return session_da_gui, ta1


def reset_session_state():
    return (
        None,   # session_id
        None,   # goi_session
        False,  # session_da_gui
        False,  # da_relay_data
        None,   # deadline
        set(),  # seen keys
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print("====================================================")
    print(" rBS - NATIVE 8 FRAME + ARQ ACK32 + FEC 8+1")
    print(" SESSION_ID64 + VOICE SEQ32")
    print(" INNER VOICE = 176 BYTE")
    print(" INNER FEC   = 184 BYTE")
    print(" RELAY VOICE = 180 BYTE")
    print(" RELAY FEC   = 188 BYTE")
    print(" FEC PARITY = XOR(ciphertext160 + original GCM tag8)")
    print(" CR = 4/5 | SF7 | BW500k")
    print(" FIRST HOP = STOP-AND-WAIT ARQ")
    print(" SECOND HOP = PACKET FEC 8 DATA + 1 PARITY")
    print(" FINAL = AUDIO_END -> END#1 -> DU PLAY_REPORT -> SU -> END#2/#3")
    print(f" PHASE2_RX_GUARD = {PHASE2_RX_GUARD * 1000:.1f} ms")
    print("====================================================")

    CS = digitalio.DigitalInOut(board.D5)
    RESET = digitalio.DigitalInOut(board.D25)

    spi = busio.SPI(
        board.SCK,
        MOSI=board.MOSI,
        MISO=board.MISO,
    )

    try:
        rfm9x = adafruit_rfm9x.RFM9x(
            spi,
            CS,
            RESET,
            TAN_SO_LORA,
            baudrate=1000000,
        )

        rfm9x.signal_bandwidth = 500000
        rfm9x.spreading_factor = 7
        rfm9x.coding_rate = 5
        rfm9x.tx_power = 23

    except RuntimeError as error:
        print(
            "[rBS ERROR] Khoi tao LoRa that bai:",
            error,
        )
        return

    print("[rBS] Khoi tao LoRa THANH CONG")
    print("[rBS] PHASE 1 -> RX SU")
    print()

    (
        session_id_hien_tai,
        goi_session,
        session_da_gui,
        da_relay_data,
        deadline_end_audio,
        seen_keys,
    ) = reset_session_state()

    t_ack_end_us = None

    while True:

        t_rx_call_us = now_us()

        if t_ack_end_us is not None:
            print(
                f"[TIME][rBS] ACK_END -> RX_CALL = "
                f"{t_rx_call_us - t_ack_end_us} us"
            )
            t_ack_end_us = None

        packet = rfm9x.receive(
            timeout=0.005,
            with_header=True,
        )

        now = time.monotonic()
        t_packet_rx_us = (
            now_us()
            if packet is not None
            else None
        )

        if packet is not None:

            info = parse_packet_su(
                packet
            )

            if info is None:
                continue

            # ----------------------------------------------------
            # AUDIO_END
            # ----------------------------------------------------
            if info["kind"] == "audio_end":

                print(
                    "[rBS RX CTRL] AUDIO_END 5B FROM SU"
                )

                if da_relay_data:
                    gui_end_audio_cho_du(
                        rfm9x,
                        session_id_hien_tai,
                    )

                    print(
                        "[rBS] SESSION AUDIO KET THUC - EXPLICIT"
                    )

                (
                    session_id_hien_tai,
                    goi_session,
                    session_da_gui,
                    da_relay_data,
                    deadline_end_audio,
                    seen_keys,
                ) = reset_session_state()

                print("[rBS] CHO SESSION MOI...")
                print()
                continue

            # ----------------------------------------------------
            # SESSION
            # ----------------------------------------------------
            if info["kind"] == "session":

                session_moi = info[
                    "session_id"
                ]

                if (
                    session_id_hien_tai
                    != session_moi
                ):
                    if da_relay_data:
                        gui_end_audio_cho_du(
                            rfm9x,
                            session_id_hien_tai,
                        )

                    session_id_hien_tai = (
                        session_moi
                    )

                    goi_session = (
                        info["raw"]
                    )

                    session_da_gui = False
                    da_relay_data = False
                    deadline_end_audio = None
                    seen_keys.clear()

                    print(
                        f"[rBS RX] NEW SESSION = "
                        f"{session_moi:016X}"
                    )

                else:
                    goi_session = (
                        info["raw"]
                    )

                    print(
                        f"[rBS RX] SESSION LAP LAI = "
                        f"{session_moi:016X}"
                    )

                continue

            # ----------------------------------------------------
            # VOICE / FEC
            # ----------------------------------------------------
            if info["kind"] in ("voice", "fec"):

                if session_id_hien_tai is None:
                    print(
                        "[rBS DROP] DATA chua co SESSION"
                    )
                    continue

                if info["kind"] == "voice":
                    key = (
                        TYPE_VOICE,
                        info["seq"],
                    )

                    print(
                        f"[rBS RX] VOICE | "
                        f"SEQ={info['seq']} | "
                        f"FRAME={info['frames']} | "
                        f"LAST={1 if info['last_audio'] else 0}"
                    )
                else:
                    key = (
                        TYPE_FEC,
                        info["group_start"],
                    )

                    print(
                        f"[rBS RX] FEC | "
                        f"GROUP_START={info['group_start']} | "
                        f"DATA={info['data_count']} | "
                        f"HAS_LAST={1 if info['has_last'] else 0} | "
                        f"FINAL_FRAME={info['final_frames']}"
                    )

                # Có data mới => chưa end trong lúc xử lý nó.
                deadline_end_audio = None

                # ------------------------------------------------
                # DUPLICATE DO ACK BỊ MẤT
                #
                # Không relay lại cho DU.
                # Chỉ phát lại đúng ACK KIND+SEQ.
                # ------------------------------------------------
                if key in seen_keys:

                    print(
                        f"[rBS ARQ] DUP -> KHONG RELAY LAI | "
                        f"KIND=0x{info['ack_kind']:02X} | "
                        f"SEQ={info['ack_seq']}"
                    )

                    time.sleep(
                        READY_GUARD
                    )

                    ta0 = now_us()

                    gui_ready_ack(
                        rfm9x,
                        info["ack_kind"],
                        info["ack_seq"],
                    )

                    t_ack_end_us = now_us()

                    print(
                        f"[TIME][rBS] DUP_ACK_DUR = "
                        f"{t_ack_end_us - ta0} us"
                    )

                    # Nếu duplicate thuộc final path, vẫn giữ fallback.
                    is_final_marker = (
                        (
                            info["kind"] == "voice"
                            and info["last_audio"]
                        )
                        or
                        (
                            info["kind"] == "fec"
                            and info["has_last"]
                        )
                    )

                    if is_final_marker:
                        deadline_end_audio = (
                            time.monotonic()
                            + END_OF_AUDIO_IDLE
                        )

                    continue

                # ------------------------------------------------
                # PACKET MỚI -> RELAY 1 LẦN
                # ------------------------------------------------
                seen_keys.add(
                    key
                )

                (
                    session_da_gui,
                    t_ack_end_us,
                ) = relay_one_data_packet(
                    rfm9x,
                    info,
                    goi_session,
                    session_da_gui,
                    t_packet_rx_us,
                )

                da_relay_data = True

                # Final VOICE còn phải có FEC sau nó.
                # Deadline chỉ là fallback dài.
                if (
                    (
                        info["kind"] == "voice"
                        and info["last_audio"]
                    )
                    or
                    (
                        info["kind"] == "fec"
                        and info["has_last"]
                    )
                ):
                    deadline_end_audio = (
                        time.monotonic()
                        + END_OF_AUDIO_IDLE
                    )

                continue

        # ========================================================
        # FALLBACK END — chỉ khi explicit AUDIO_END mất/hỏng
        # ========================================================

        if (
            deadline_end_audio is not None
            and
            now >= deadline_end_audio
        ):
            print(
                "[rBS FALLBACK] END_OF_AUDIO_IDLE -> DU"
            )

            if da_relay_data:
                gui_end_audio_cho_du(
                    rfm9x,
                    session_id_hien_tai,
                )

            (
                session_id_hien_tai,
                goi_session,
                session_da_gui,
                da_relay_data,
                deadline_end_audio,
                seen_keys,
            ) = reset_session_state()

            print("[rBS] CHO SESSION MOI...")
            print()


if __name__ == "__main__":
    main()
