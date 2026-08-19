import time
import board
import busio
import digitalio
import adafruit_rfm9x


# ============================================================
# ĐỒNG HỒ ĐO THỜI GIAN
#
# Dùng monotonic_ns() để tránh ảnh hưởng nếu giờ hệ thống thay đổi.
# Kết quả quy đổi về microsecond (us).
# ============================================================

def now_us():
    return time.monotonic_ns() // 1000


# ============================================================
# ID TRẠM
# ============================================================

ID_TRAM_SU = 0x01
ID_TRAM_DU = 0x02
ID_TRAM_RBS = 0x03


# ============================================================
# LORA
# ============================================================

TAN_SO_LORA = 433.0

BURST_SIZE = 2

# Fallback khi burst bị thiếu VOICE do packet loss.
# Bình thường LAST_AUDIO trong VOICE làm final burst relay ngay.
FINAL_BURST_TIMEOUT = 0.060

# Sau khi rBS gửi READY mà không thấy VOICE tiếp theo trong
# khoảng này, coi câu thoại đã kết thúc và báo END cho DU.
END_OF_AUDIO_IDLE = 0.120

# Khoảng nghỉ giữa 2 packet RELAY liên tiếp.
# rfm9x.send() chỉ đảm bảo TX của rBS đã xong; DU vẫn cần
# một khoảng ngắn để parse packet và quay lại RX continuous.
# Vì hệ thống đang "ghi trước -> gửi sau", ưu tiên độ tin cậy.
RELAY_PACKET_GAP = 0.010

# Sau packet relay cuối, SU cần một khoảng rất ngắn để
# xử lý packet 100B nghe ké từ rBS->DU và quay lại receive()
# trước khi rBS phát READY.
READY_GUARD = 0.010

# ============================================================
# PHASE 1 -> PHASE 2 GUARD
#
# DU cũng nghe trực tiếp packet 96B của SU trong PHASE 1.
# Sau packet cuối burst, DU cần một khoảng rất ngắn để:
#   - đọc/xả packet SU trực tiếp
#   - gọi LoRa.receive() trở lại
#
# Nếu rBS relay packet đầu của PHASE 2 ngay lập tức,
# DU có thể bỏ lỡ packet đầu đó.
#
# Chỉ áp dụng từ burst thứ 2 trở đi.
# Burst đầu đã có SESSION_START relay làm khoảng đệm tự nhiên.
# ============================================================

PHASE2_RX_GUARD = 0.008


# ============================================================
# TYPE BÊN TRONG PACKET SU
# ============================================================

TYPE_VOICE = 0x01
TYPE_SESSION_START = 0x02
TYPE_READY = 0x03

# Control explicit từ SU.
# BURST_END riêng đã bỏ; final packet được đánh dấu bằng LAST_AUDIO
# trong byte 3 của VOICE (thuộc AAD).
TYPE_AUDIO_END_SU = 0x04

SIZE_VOICE = 96
SIZE_SESSION = 12

# Control SU phải có ít nhất 5 byte để Adafruit RFM9x
# không loại packet trước khi trả về Python.
#
# Byte 0..3 = control header hiện tại.
# Byte 4    = dummy 0x00.
SIZE_CONTROL_SU = 5

VOICE_LENGTH = 88
VOICE_LENGTH_MASK = 0x7F
FLAG_LAST_AUDIO = 0x80


# ============================================================
# TYPE WRAPPER rBS -> DU
#
# Trên không khí:
#
# RELAY:
#   byte 0 = DST  = DU
#   byte 1 = SRC  = rBS
#   byte 2 = TYPE_RELAY
#   byte 3 = độ dài packet gốc (12 hoặc 96)
#   byte 4... = packet SU nguyên vẹn
#
# SESSION: 4 + 12 = 16 byte
# VOICE:   4 + 96 = 100 byte
#
# AES-GCM vẫn end-to-end vì packet 96 byte gốc nằm nguyên vẹn
# bên trong wrapper.
# ============================================================

TYPE_RELAY = 0x10
TYPE_RELAY_END = 0x11


# ============================================================
# PARSER PACKET GỐC TỪ SU
# ============================================================

def parse_packet_su(packet_bytes):

    if packet_bytes is None:
        return None

    packet_bytes = bytes(packet_bytes)

    if len(packet_bytes) not in (
        SIZE_SESSION,
        SIZE_VOICE,
        SIZE_CONTROL_SU
    ):
        return None

    dst = packet_bytes[0]
    src = packet_bytes[1]
    packet_type = packet_bytes[2] & 0x3F
    length_field = packet_bytes[3]

    # Chỉ nhận luồng SU -> DU.
    # rBS là relay trung gian nhưng packet gốc vẫn có DST = DU.
    if dst != ID_TRAM_DU or src != ID_TRAM_SU:
        return None

    # --------------------------------------------------------
    # CONTROL EXPLICIT 5 BYTE TỪ SU
    #
    # Adafruit RFM9x reject packet < 5 byte, nên AUDIO_END
    # có thêm byte dummy ở cuối.
    #
    # Byte 0..3 giữ format control cũ.
    # Byte 4 là dummy và không tham gia parse.
    #
    # Chỉ còn AUDIO_END sau READY.
    # BURST_END riêng đã được thay bằng LAST_AUDIO trong VOICE.
    # --------------------------------------------------------
    if len(packet_bytes) == SIZE_CONTROL_SU:

        if packet_type == TYPE_AUDIO_END_SU:
            return {
                "kind": "audio_end",
                "raw": packet_bytes,
            }

        return None

    if packet_type == TYPE_SESSION_START:

        if len(packet_bytes) != SIZE_SESSION:
            return None

        if length_field != 8:
            return None

        session_id = int.from_bytes(
            packet_bytes[4:12],
            byteorder="big",
            signed=False
        )

        if session_id == 0:
            return None

        return {
            "kind": "session",
            "raw": packet_bytes,
            "session_id": session_id,
        }

    if packet_type == TYPE_VOICE:

        if len(packet_bytes) != SIZE_VOICE:
            return None

        voice_length = (
            length_field
            & VOICE_LENGTH_MASK
        )

        la_packet_cuoi = (
            length_field
            & FLAG_LAST_AUDIO
        ) != 0

        if voice_length != VOICE_LENGTH:
            return None

        seq = int.from_bytes(
            packet_bytes[4:8],
            byteorder="big",
            signed=False
        )

        so_frame = ((packet_bytes[2] >> 6) & 0x03) + 1

        return {
            "kind": "voice",
            "raw": packet_bytes,
            "seq": seq,
            "frames": so_frame,
            "last_audio": la_packet_cuoi,
        }

    return None


# ============================================================
# GỬI WRAPPER RELAY rBS -> DU
# ============================================================

def gui_relay_wrapper(rfm9x, packet_goc):

    packet_goc = bytes(packet_goc)

    if len(packet_goc) not in (SIZE_SESSION, SIZE_VOICE):
        raise ValueError(
            f"Do dai packet goc khong hop le: {len(packet_goc)}"
        )

    # Adafruit RFM9x tự thêm 4-byte RadioHead header.
    # packet_goc được dùng làm payload nguyên vẹn.
    rfm9x.send(
        packet_goc,
        destination=ID_TRAM_DU,
        node=ID_TRAM_RBS,
        identifier=TYPE_RELAY,
        flags=len(packet_goc)
    )


# ============================================================
# READY rBS -> SU
#
# send() không cho payload rỗng nên dùng 1 byte 0x00.
# Tổng packet trên không khí = 5 byte.
# ============================================================

def gui_ready_cho_su(rfm9x):

    rfm9x.send(
        b"\x00",
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_READY,
        flags=0
    )

    print("[rBS CTRL] READY -> SU")


# ============================================================
# END AUDIO rBS -> DU
#
# Báo cho DU rằng không còn burst mới của câu hiện tại.
# DU sẽ bắt đầu phát toàn bộ audio đã buffer.
# ============================================================

def gui_end_audio_cho_du(rfm9x):

    # Gửi lặp để giảm nguy cơ mất control packet.
    for lan in range(3):

        rfm9x.send(
            b"\x00",
            destination=ID_TRAM_DU,
            node=ID_TRAM_RBS,
            identifier=TYPE_RELAY_END,
            flags=0
        )

        if lan < 2:
            time.sleep(0.020)

    print("[rBS CTRL] END_AUDIO -> DU")


# ============================================================
# RELAY MỘT BURST
# ============================================================

def relay_burst(
    rfm9x,
    burst_voice,
    goi_session,
    session_da_gui,
    t_rx_last_us
):

    if not burst_voice:
        return session_da_gui

    print()
    print("----------------------------------------------------")
    print(
        f"[rBS] PHASE 2 BAT DAU | RELAY {len(burst_voice)} PACKET"
    )
    print("----------------------------------------------------")


    # ========================================================
    # ĐO CHUYỂN PHASE 1 -> PHASE 2
    #
    # t_rx_last_us = thời điểm rBS vừa nhận xong packet cuối burst.
    # t_phase2_start_us = thời điểm code bắt đầu pha relay.
    # ========================================================

    t_phase2_start_us = now_us()

    if t_rx_last_us is not None:
        print(
            f"[TIME][rBS] RX_LAST -> PHASE2_START = "
            f"{t_phase2_start_us - t_rx_last_us} us"
        )


    # ========================================================
    # GUARD CHO DU TRƯỚC PACKET VOICE ĐẦU PHASE 2
    #
    # Từ burst thứ 2 trở đi, không còn SESSION_START đứng trước
    # VOICE. Nếu relay ngay, DU có thể vẫn đang xả packet SU
    # trực tiếp 96B của PHASE 1 và bỏ lỡ packet relay đầu tiên.
    #
    # Burst đầu không cần guard này vì SESSION_START relay đã
    # tạo khoảng đệm trước SEQ 0.
    # ========================================================

    if session_da_gui:

        t_phase2_guard_start_us = now_us()

        time.sleep(
            PHASE2_RX_GUARD
        )

        t_phase2_guard_end_us = now_us()

        print(
            f"[TIME][rBS] PHASE2_RX_GUARD = "
            f"{(t_phase2_guard_end_us - t_phase2_guard_start_us) / 1000:.3f} ms"
        )


    # --------------------------------------------------------
    # SESSION phải đi qua rBS vì DU từ giờ bỏ packet SU trực tiếp.
    # Chỉ gửi trước burst VOICE đầu tiên của session.
    # --------------------------------------------------------

    if (
        not session_da_gui
        and goi_session is not None
    ):
        session_id = int.from_bytes(
            goi_session[4:12],
            byteorder="big",
            signed=False
        )

        # Gửi 2 lần cho chắc chắn hơn.
        for lan in range(2):
            gui_relay_wrapper(
                rfm9x,
                goi_session
            )

            if lan == 0:
                # Chỉ cần guard nhỏ giữa hai SESSION relay.
                time.sleep(0.005)

        print(
            f"[rBS TX RELAY] SESSION_START | "
            f"SESSION = {session_id:016X}"
        )

        session_da_gui = True


    # --------------------------------------------------------
    # VOICE
    # --------------------------------------------------------

    for index, packet_voice in enumerate(burst_voice):

        seq = int.from_bytes(
            packet_voice[4:8],
            byteorder="big",
            signed=False
        )

        so_frame = (
            ((packet_voice[2] >> 6) & 0x03)
            + 1
        )


        # ====================================================
        # ĐO THỜI GIAN PHÁT TỪNG PACKET
        # ====================================================

        t_tx_start_us = now_us()

        gui_relay_wrapper(
            rfm9x,
            packet_voice
        )

        t_tx_end_us = now_us()


        print(
            f"[rBS TX RELAY] VOICE | "
            f"SEQ = {seq} | FRAME = {so_frame}"
        )

        print(
            f"[TIME][rBS TX] SEQ={seq} | "
            f"START={t_tx_start_us} us | "
            f"END={t_tx_end_us} us | "
            f"TX_DUR={t_tx_end_us - t_tx_start_us} us"
        )


        # ====================================================
        # GAP GIỮA HAI PACKET
        #
        # Chỉ delay nếu CÒN packet kế tiếp.
        # Packet cuối burst không cần delay trước READY.
        # ====================================================

        if index < len(burst_voice) - 1:

            t_gap_start_us = now_us()

            time.sleep(
                RELAY_PACKET_GAP
            )

            t_gap_end_us = now_us()

            print(
                f"[TIME][rBS GAP] AFTER_SEQ={seq} | "
                f"TARGET={RELAY_PACKET_GAP * 1000:.1f} ms | "
                f"ACTUAL={(t_gap_end_us - t_gap_start_us) / 1000:.3f} ms"
            )

        else:

            print(
                f"[TIME][rBS GAP] AFTER_SEQ={seq} | "
                f"LAST_PACKET -> NO GAP"
            )


    # ========================================================
    # VOICE PHASE 2 KẾT THÚC
    # ========================================================

    t_voice_phase2_end_us = now_us()

    print(
        f"[TIME][rBS] PHASE2_VOICE_DUR = "
        f"{t_voice_phase2_end_us - t_phase2_start_us} us "
        f"({(t_voice_phase2_end_us - t_phase2_start_us) / 1000:.3f} ms)"
    )


    # ========================================================
    # READY GUARD
    #
    # Không phải gap cho DU.
    # Mục đích: cho SU đủ thời gian xử lý packet relay cuối
    # mà nó nghe được trên cùng tần số, rồi quay lại RX để
    # bắt được READY.
    # ========================================================

    t_ready_guard_start_us = now_us()

    time.sleep(
        READY_GUARD
    )

    t_ready_guard_end_us = now_us()

    print(
        f"[TIME][rBS] READY_GUARD = "
        f"{(t_ready_guard_end_us - t_ready_guard_start_us) / 1000:.3f} ms"
    )


    # ========================================================
    # READY
    # ========================================================

    t_ready_start_us = now_us()

    gui_ready_cho_su(
        rfm9x
    )

    t_ready_end_us = now_us()

    print(
        f"[TIME][rBS] READY_TX_DUR = "
        f"{t_ready_end_us - t_ready_start_us} us "
        f"({(t_ready_end_us - t_ready_start_us) / 1000:.3f} ms)"
    )


    print("[rBS] PHASE 2 KET THUC")
    print("[rBS] PHASE 1 -> QUAY LAI RX SU")
    print("----------------------------------------------------")
    print()


    # Trả thêm timestamp để main đo READY_END -> RX_CALL.
    return session_da_gui, t_ready_end_us


# ============================================================
# MAIN
# ============================================================

def main():

    print("====================================================")
    print(" rBS - HALF DUPLEX BURST RELAY + WRAPPER")
    print(" SESSION_ID 64-bit + SEQ32")
    print(f" BURST_SIZE = {BURST_SIZE} PACKET")
    print(" INNER VOICE = 96 BYTE")
    print(" RELAY VOICE = 100 BYTE")
    print(" LOW LATENCY CTRL = LAST_AUDIO(AAD)")
    print(" FINAL CTRL = READY -> SU AUDIO_END(5B) -> DU END x3")
    print(f" CONTROL SU = {SIZE_CONTROL_SU} BYTE")
    print(f" PHASE2_RX_GUARD = {PHASE2_RX_GUARD * 1000:.1f} ms")
    print("====================================================")


    # --------------------------------------------------------
    # PHẦN CỨNG
    # Giữ đúng chân đang dùng trên Raspberry Pi.
    # --------------------------------------------------------

    CS = digitalio.DigitalInOut(
        board.D5
    )

    RESET = digitalio.DigitalInOut(
        board.D25
    )

    spi = busio.SPI(
        board.SCK,
        MOSI=board.MOSI,
        MISO=board.MISO
    )


    try:

        rfm9x = adafruit_rfm9x.RFM9x(
            spi,
            CS,
            RESET,
            TAN_SO_LORA,
            baudrate=1000000
        )

        rfm9x.signal_bandwidth = 500000
        rfm9x.spreading_factor = 7
        rfm9x.coding_rate = 5

        # Giữ giá trị đang chạy trên hệ thống của bạn.
        rfm9x.tx_power = 23

    except RuntimeError as error:

        print(
            "[rBS ERROR] Khoi tao LoRa that bai:",
            error
        )

        return


    print("[rBS] Khoi tao LoRa THANH CONG")
    print("[rBS] PHASE 1 -> RX SU")
    print()


    # --------------------------------------------------------
    # STATE
    # --------------------------------------------------------

    session_id_hien_tai = None

    goi_session = None

    session_da_gui = False

    burst_voice = []

    thoi_diem_voice_cuoi = None

    # Sau mỗi lần relay + READY, rBS chờ burst tiếp.
    # Nếu hết hạn mà không có VOICE mới => END_AUDIO.
    deadline_end_audio = None

    da_relay_voice_trong_session = False

    # Timestamp packet VOICE cuối cùng của burst vừa nhận.
    t_rx_last_us = None

    # Timestamp lúc READY vừa phát xong.
    # Dùng để đo thời gian từ TX/control -> quay lại gọi RX.
    t_ready_end_us = None


    while True:

        # ====================================================
        # ĐO THỜI ĐIỂM QUAY LẠI RX
        # ====================================================

        t_rx_call_us = now_us()

        if t_ready_end_us is not None:
            print(
                f"[TIME][rBS] READY_END -> RX_CALL = "
                f"{t_rx_call_us - t_ready_end_us} us"
            )

            t_ready_end_us = None


        packet = rfm9x.receive(
            timeout=0.005,
            with_header=True
        )

        now = time.monotonic()

        # Nếu có packet thì đây xấp xỉ thời điểm rBS vừa nhận xong
        # packet đó từ radio / thư viện.
        t_packet_rx_us = now_us() if packet is not None else None


        # ====================================================
        # CÓ PACKET
        # ====================================================

        if packet is not None:

            thong_tin = parse_packet_su(
                packet
            )

            if thong_tin is None:
                continue


            # ------------------------------------------------
            # AUDIO_END EXPLICIT
            #
            # SU gửi sau READY của burst cuối.
            # LAST_AUDIO đã giúp rBS relay final burst ngay;
            # AUDIO_END là tín hiệu chính để rBS báo DU PLAY.
            # ------------------------------------------------

            if thong_tin["kind"] == "audio_end":

                print(
                    "[rBS RX CTRL] AUDIO_END 5B FROM SU"
                )

                # Safety: nếu vẫn còn packet cuối chưa relay vì một
                # lý do bất thường thì relay trước khi END.
                if burst_voice:

                    session_da_gui, t_ready_end_us = relay_burst(
                        rfm9x,
                        burst_voice,
                        goi_session,
                        session_da_gui,
                        t_rx_last_us
                    )

                    burst_voice.clear()

                    t_rx_last_us = None

                    thoi_diem_voice_cuoi = None

                    da_relay_voice_trong_session = True


                if da_relay_voice_trong_session:

                    gui_end_audio_cho_du(
                        rfm9x
                    )

                    print(
                        "[rBS] SESSION AUDIO KET THUC - EXPLICIT"
                    )

                    print(
                        "[rBS] CHO SESSION MOI..."
                    )

                    print()


                    session_id_hien_tai = None

                    goi_session = None

                    session_da_gui = False

                    burst_voice.clear()

                    thoi_diem_voice_cuoi = None

                    deadline_end_audio = None

                    da_relay_voice_trong_session = False

                    t_rx_last_us = None
                    t_ready_end_us = None

                continue


            # ------------------------------------------------
            # SESSION_START
            # ------------------------------------------------

            if thong_tin["kind"] == "session":

                session_moi = thong_tin[
                    "session_id"
                ]

                if (
                    session_id_hien_tai
                    != session_moi
                ):
                    # Nếu session trước đã có voice nhưng vì lý do
                    # nào đó chưa gửi END, đóng nó trước.
                    if da_relay_voice_trong_session:
                        gui_end_audio_cho_du(
                            rfm9x
                        )

                    session_id_hien_tai = (
                        session_moi
                    )

                    goi_session = (
                        thong_tin["raw"]
                    )

                    session_da_gui = False

                    burst_voice.clear()

                    thoi_diem_voice_cuoi = None

                    deadline_end_audio = None

                    da_relay_voice_trong_session = False

                    t_rx_last_us = None
                    t_ready_end_us = None

                    print(
                        f"[rBS RX] NEW SESSION = "
                        f"{session_moi:016X}"
                    )

                else:

                    # Giữ bản session mới nhất nhưng không reset.
                    goi_session = (
                        thong_tin["raw"]
                    )

                    print(
                        f"[rBS RX] SESSION LAP LAI = "
                        f"{session_moi:016X}"
                    )

                continue


            # ------------------------------------------------
            # VOICE
            # ------------------------------------------------

            if thong_tin["kind"] == "voice":

                # Khi packet mới của burst kế tiếp đã tới,
                # chắc chắn câu chưa kết thúc.
                deadline_end_audio = None

                seq = thong_tin["seq"]

                so_frame = thong_tin[
                    "frames"
                ]

                la_packet_cuoi = thong_tin[
                    "last_audio"
                ]

                if la_packet_cuoi:
                    print(
                        "[rBS] LAST_AUDIO NHAN DUOC -> FINAL BURST RELAY NGAY"
                    )

                print(
                    f"[rBS RX] VOICE | "
                    f"SEQ = {seq} | "
                    f"FRAME = {so_frame} | "
                    f"LAST = {1 if la_packet_cuoi else 0}"
                )

                print(
                    f"[TIME][rBS RX] SEQ={seq} | "
                    f"T={t_packet_rx_us} us"
                )

                burst_voice.append(
                    thong_tin["raw"]
                )

                thoi_diem_voice_cuoi = now

                # Luôn ghi lại thời điểm packet VOICE vừa nhận.
                # Khi burst đủ 4, đây chính là thời điểm nhận xong
                # packet cuối của Phase 1.
                t_rx_last_us = t_packet_rx_us

                print(
                    f"[rBS BUFFER] "
                    f"{len(burst_voice)}/{BURST_SIZE}"
                )


                # --------------------------------------------
                # RELAY KHI:
                #   - đủ BURST_SIZE = 2 packet; hoặc
                #   - gặp LAST_AUDIO, kể cả final burst chỉ có 1 packet.
                # --------------------------------------------

                if (
                    len(burst_voice)
                    >= BURST_SIZE
                    or
                    la_packet_cuoi
                ):

                    session_da_gui, t_ready_end_us = relay_burst(
                        rfm9x,
                        burst_voice,
                        goi_session,
                        session_da_gui,
                        t_rx_last_us
                    )

                    burst_voice.clear()

                    # Burst đã relay xong.
                    t_rx_last_us = None

                    da_relay_voice_trong_session = True

                    deadline_end_audio = (
                        time.monotonic()
                        +
                        END_OF_AUDIO_IDLE
                    )


        # ====================================================
        # KHÔNG CÓ PACKET / KIỂM TRA TIMEOUT
        # ====================================================

        now = time.monotonic()


        # ----------------------------------------------------
        # FALLBACK BURST THIẾU PACKET
        #
        # Bình thường LAST_AUDIO làm final burst relay ngay.
        # Nhánh 60 ms chỉ còn cứu burst bị thiếu VOICE do packet loss.
        # ----------------------------------------------------

        if (
            burst_voice
            and
            thoi_diem_voice_cuoi is not None
            and
            (
                now
                -
                thoi_diem_voice_cuoi
            )
            >= FINAL_BURST_TIMEOUT
        ):

            session_da_gui, t_ready_end_us = relay_burst(
                rfm9x,
                burst_voice,
                goi_session,
                session_da_gui,
                t_rx_last_us
            )

            burst_voice.clear()

            t_rx_last_us = None

            da_relay_voice_trong_session = True

            deadline_end_audio = (
                time.monotonic()
                +
                END_OF_AUDIO_IDLE
            )


        # ----------------------------------------------------
        # END AUDIO
        #
        # Chỉ được kích hoạt sau khi đã relay ít nhất 1 burst.
        # Nếu SU bắt đầu burst mới trước deadline thì nhánh RX
        # phía trên đã đặt deadline_end_audio = None.
        # ----------------------------------------------------

        if (
            da_relay_voice_trong_session
            and
            deadline_end_audio is not None
            and
            now >= deadline_end_audio
        ):

            gui_end_audio_cho_du(
                rfm9x
            )

            print(
                "[rBS] SESSION AUDIO KET THUC"
            )

            print(
                "[rBS] CHO SESSION MOI..."
            )

            print()


            session_id_hien_tai = None

            goi_session = None

            session_da_gui = False

            burst_voice.clear()

            thoi_diem_voice_cuoi = None

            deadline_end_audio = None

            da_relay_voice_trong_session = False

            t_rx_last_us = None
            t_ready_end_us = None


if __name__ == "__main__":

    try:
        main()

    except KeyboardInterrupt:
        print(
            "\n[rBS] Dung chuong trinh."
        )