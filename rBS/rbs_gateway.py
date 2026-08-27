import time
import board
import busio
import digitalio
import adafruit_rfm9x

from gps_rbs import (
    QuanLyGPSRBS,
    parse_gps_report,
    TYPE_GPS_REPORT,
    SIZE_GPS_REPORT,
    ID_TRAM_SU as GPS_ID_SU,
    ID_TRAM_DU as GPS_ID_DU,
)


def now_us():
    return time.monotonic_ns() // 1000


def lay_rssi_snr(rfm9x):
    """Đọc RSSI/SNR của gói vừa nhận. Nếu thư viện không hỗ trợ thì trả None."""
    rssi = None
    snr = None

    try:
        value = getattr(rfm9x, "last_rssi", None)
        if value is not None:
            rssi = float(value)
    except Exception:
        rssi = None

    try:
        value = getattr(rfm9x, "last_snr", None)
        if value is not None:
            snr = float(value)
    except Exception:
        snr = None

    return rssi, snr


def chuoi_rssi_snr(rssi, snr):
    rssi_text = "N/A" if rssi is None else f"{rssi:.1f} dBm"
    snr_text = "N/A" if snr is None else f"{snr:.1f} dB"
    return f"RSSI={rssi_text} | SNR={snr_text}"


def tao_thong_ke_lien_ket():
    return {
        "so_goi_rx": 0,
        "rssi_count": 0,
        "rssi_sum": 0.0,
        "rssi_min": None,
        "rssi_max": None,
        "snr_count": 0,
        "snr_sum": 0.0,
        "snr_min": None,
        "snr_max": None,
        "theo_loai": {},
    }


def cap_nhat_thong_ke_lien_ket(thong_ke, rssi, snr, loai_goi):
    if thong_ke is None:
        return

    thong_ke["so_goi_rx"] += 1
    thong_ke["theo_loai"][loai_goi] = thong_ke["theo_loai"].get(loai_goi, 0) + 1

    if rssi is not None:
        thong_ke["rssi_count"] += 1
        thong_ke["rssi_sum"] += float(rssi)
        thong_ke["rssi_min"] = (
            float(rssi)
            if thong_ke["rssi_min"] is None
            else min(thong_ke["rssi_min"], float(rssi))
        )
        thong_ke["rssi_max"] = (
            float(rssi)
            if thong_ke["rssi_max"] is None
            else max(thong_ke["rssi_max"], float(rssi))
        )

    if snr is not None:
        thong_ke["snr_count"] += 1
        thong_ke["snr_sum"] += float(snr)
        thong_ke["snr_min"] = (
            float(snr)
            if thong_ke["snr_min"] is None
            else min(thong_ke["snr_min"], float(snr))
        )
        thong_ke["snr_max"] = (
            float(snr)
            if thong_ke["snr_max"] is None
            else max(thong_ke["snr_max"], float(snr))
        )


def _fmt_tb(sum_value, count, don_vi):
    if count <= 0:
        return "N/A"
    return f"{sum_value / count:.1f} {don_vi}"


def _fmt_minmax(value, don_vi):
    if value is None:
        return "N/A"
    return f"{value:.1f} {don_vi}"


def _chuoi_dem_theo_loai(thong_ke):
    if not thong_ke["theo_loai"]:
        return "không có"
    return ", ".join(
        f"{ten}={so_luong}"
        for ten, so_luong in sorted(thong_ke["theo_loai"].items())
    )


def in_mot_lien_ket(ten, thong_ke):
    print(f"[{ten}]")
    print(
        f"  GÓI NHẬN TẠI rBS = {thong_ke['so_goi_rx']} | "
        f"CHI TIẾT: {_chuoi_dem_theo_loai(thong_ke)}"
    )
    print(
        "  RSSI: "
        f"TB={_fmt_tb(thong_ke['rssi_sum'], thong_ke['rssi_count'], 'dBm')} | "
        f"MIN={_fmt_minmax(thong_ke['rssi_min'], 'dBm')} | "
        f"MAX={_fmt_minmax(thong_ke['rssi_max'], 'dBm')} | "
        f"MẪU={thong_ke['rssi_count']}"
    )
    print(
        "  SNR : "
        f"TB={_fmt_tb(thong_ke['snr_sum'], thong_ke['snr_count'], 'dB')} | "
        f"MIN={_fmt_minmax(thong_ke['snr_min'], 'dB')} | "
        f"MAX={_fmt_minmax(thong_ke['snr_max'], 'dB')} | "
        f"MẪU={thong_ke['snr_count']}"
    )


def in_tong_ket_lien_ket(diag, ly_do, gps_manager=None):
    if diag is None:
        return

    print()
    print("====================================================")
    print(" TỔNG KẾT CHẤT LƯỢNG LIÊN KẾT THEO SESSION")
    print("====================================================")
    print(f"SESSION = {diag['session_id']:016X}")
    print(f"KẾT THÚC/CHỐT SỐ LIỆU DO = {ly_do}")
    print("CẤU HÌNH = 433 MHz | SF7 | BW500 kHz | CR 4/5 | rBS TX=23 dBm")
    print()

    in_mot_lien_ket("SU -> rBS", diag["link_su_rbs"])
    print(
        f"  ARQ RETRY QUAN SÁT TẠI rBS = {diag['so_goi_trung_arq']} | "
        f"SU GỬI LẠI SESSION_START = {diag['so_lan_su_gui_lai_start']}"
    )
    print(
        "  LƯU Ý: gói ARQ trùng có thể do DATA SU->rBS cần gửi lại, "
        "hoặc do ACK rBS->SU bị mất; không được coi trực tiếp là PER SU->rBS."
    )
    print()

    in_mot_lien_ket("DU -> rBS", diag["link_du_rbs"])
    print(
        f"  SESSION_START rBS->DU ĐÃ THỬ = {diag['so_lan_start_toi_du']} | "
        f"SESSION_READY DU->rBS = {1 if diag['da_nhan_ready_tu_du'] else 0}"
    )
    print()

    su = diag["link_su_rbs"]
    du = diag["link_du_rbs"]
    if su["rssi_count"] > 0 and du["rssi_count"] > 0:
        su_avg = su["rssi_sum"] / su["rssi_count"]
        du_avg = du["rssi_sum"] / du["rssi_count"]
        print(
            f"[SO SÁNH RSSI TB] SU->rBS={su_avg:.1f} dBm | "
            f"DU->rBS={du_avg:.1f} dBm | CHÊNH={abs(su_avg-du_avg):.1f} dB"
        )
    if su["snr_count"] > 0 and du["snr_count"] > 0:
        su_avg = su["snr_sum"] / su["snr_count"]
        du_avg = du["snr_sum"] / du["snr_count"]
        print(
            f"[SO SÁNH SNR TB]  SU->rBS={su_avg:.1f} dB | "
            f"DU->rBS={du_avg:.1f} dB | CHÊNH={abs(su_avg-du_avg):.1f} dB"
        )

    print(
        "[GHI CHÚ] RSSI/SNR dùng để đánh giá và so sánh chất lượng link. "
        "Không quy đổi trực tiếp một giá trị RSSI thành khoảng cách mét nếu chưa hiệu chuẩn thực địa."
    )

    if gps_manager is not None:
        print()
        gps_manager.in_tom_tat(diag["session_id"])
        gps_manager.ghi_csv_session(diag, ly_do)

    print("====================================================")
    print()


def tao_chan_doan_session(session_id):
    return {
        "session_id": session_id,
        "so_lan_start_toi_du": 0,
        "da_nhan_ready_tu_du": False,
        "so_lan_ready_toi_su": 0,
        "so_lan_su_gui_lai_start": 0,
        "da_nhan_voice_dau_tien": False,
        "so_goi_trung_arq": 0,
        "link_su_rbs": tao_thong_ke_lien_ket(),
        "link_du_rbs": tao_thong_ke_lien_ket(),
        "hmi_ket_qua": "",
    }


def in_tom_tat_chan_doan(diag):
    if diag is None:
        return

    print(
        "[THỐNG KÊ BẮT TAY] "
        f"START_rBS->DU={diag['so_lan_start_toi_du']} | "
        f"READY_DU->rBS={1 if diag['da_nhan_ready_tu_du'] else 0} | "
        f"READY_rBS->SU={diag['so_lan_ready_toi_su']} | "
        f"SU_GỬI_LẠI_START={diag['so_lan_su_gui_lai_start']} | "
        f"VOICE_ĐẦU={1 if diag['da_nhan_voice_dau_tien'] else 0}"
    )


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

# HMI user feedback
TYPE_USER_RESPONSE = 0x13
TYPE_USER_CONFIRM = 0x14
TYPE_SESSION_READY = 0x15
TYPE_SESSION_FAIL = 0x16
SIZE_USER_HMI = 12
SIZE_SESSION_CTRL = 12
USER_RESPONSE_ACK = 0x01
USER_RESPONSE_NACK = 0x02
USER_HMI_FORWARD_GUARD = 0.008

# SESSION_START -> SESSION_READY handshake.
# rBS tu retry SESSION_START toi DU toi da 3 lan.
SESSION_MAX_ATTEMPTS = 3
SESSION_READY_WINDOW = 0.200
SESSION_READY_FORWARD_GUARD = 0.008

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
        SIZE_GPS_REPORT,
    ):
        return None

    dst = p[0]
    src = p[1]

    # GPS_REPORT dung TYPE 0x17 va dich truc tiep rBS, khong dung TYPE_MASK.
    if len(p) == SIZE_GPS_REPORT and p[2] == TYPE_GPS_REPORT:
        gps = parse_gps_report(p)
        if gps is not None and gps["src"] == GPS_ID_SU:
            return {"kind": "gps", "raw": p, "gps": gps}
        return None

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
        f"[rBS GỬI ACK] -> SU | "
        f"LOẠI=0x{ack_kind:02X} | SEQ={ack_seq}"
    )


def parse_session_ready_du(packet_bytes, expected_session_id):

    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)

    if len(p) != SIZE_SESSION_CTRL:
        return None

    if (
        p[0] != ID_TRAM_RBS
        or p[1] != ID_TRAM_DU
        or p[2] != TYPE_SESSION_READY
    ):
        return None

    sid = int.from_bytes(
        p[4:12],
        byteorder="big",
        signed=False,
    )

    if sid == 0:
        return None

    if expected_session_id is not None and sid != expected_session_id:
        return None

    return sid


def gui_session_ready_cho_su(rfm9x, session_id):

    rfm9x.send(
        int(session_id).to_bytes(8, byteorder="big", signed=False),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_SESSION_READY,
        flags=0,
    )

    print(
        f"[rBS GỬI] SESSION_READY -> SU | SESSION={session_id:016X}"
    )


def gui_session_fail_cho_su(rfm9x, session_id):

    rfm9x.send(
        int(session_id).to_bytes(8, byteorder="big", signed=False),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_SESSION_FAIL,
        flags=0,
    )

    print(
        f"[rBS GỬI] SESSION_FAIL -> SU | SESSION={session_id:016X}"
    )


def thiet_lap_session_voi_du(rfm9x, goi_session, session_id, diag_session=None, gps_manager=None):
    """
    rBS tự động bắt tay với DU:
      SESSION_START -> chờ SESSION_READY.
    Tổng cộng tối đa 3 lần gửi SESSION_START.

    Trả về dict để main dùng cho chẩn đoán.
    """

    ket_qua = {
        "ok": False,
        "so_lan_start_toi_du": 0,
        "da_nhan_ready_tu_du": False,
        "so_lan_ready_toi_su": 0,
    }

    for attempt in range(1, SESSION_MAX_ATTEMPTS + 1):
        ket_qua["so_lan_start_toi_du"] += 1

        print(
            f"[CHẨN ĐOÁN S1] rBS -> DU: ĐÃ PHÁT SESSION_START | "
            f"LẦN={attempt}/{SESSION_MAX_ATTEMPTS} | "
            f"SESSION={session_id:016X}"
        )

        gui_relay_wrapper(
            rfm9x,
            goi_session,
        )

        deadline = time.monotonic() + SESSION_READY_WINDOW

        while time.monotonic() < deadline:
            p = rfm9x.receive(
                timeout=0.005,
                with_header=True,
            )

            if p is None:
                continue

            rssi, snr = lay_rssi_snr(rfm9x)

            gps_report = parse_gps_report(p)
            if (
                gps_report is not None
                and gps_report["src"] == GPS_ID_DU
                and gps_report["session_id"] == session_id
            ):
                if gps_manager is not None:
                    gps_manager.cap_nhat(gps_report, rssi, snr)
                if diag_session is not None:
                    cap_nhat_thong_ke_lien_ket(
                        diag_session["link_du_rbs"],
                        rssi,
                        snr,
                        "GPS_REPORT",
                    )
                print(
                    f"[rBS GPS] NHẬN GPS_REPORT từ DU | SESSION={session_id:016X} | "
                    f"VALID={1 if gps_report['gps_valid'] else 0} | "
                    f"LAT={gps_report['latitude']:.7f} | LON={gps_report['longitude']:.7f} | "
                    f"SAT={gps_report['satellites']} | HDOP={gps_report['hdop']:.2f} | "
                    f"AGE={gps_report['fix_age_ms']} ms | {chuoi_rssi_snr(rssi, snr)}"
                )
                continue

            sid = parse_session_ready_du(
                p,
                session_id,
            )

            if sid is None:
                continue

            ket_qua["da_nhan_ready_tu_du"] = True

            if diag_session is not None:
                cap_nhat_thong_ke_lien_ket(
                    diag_session["link_du_rbs"],
                    rssi,
                    snr,
                    "SESSION_READY",
                )

            print(
                f"[CHẨN ĐOÁN S2] DU -> rBS: NHẬN SESSION_READY = OK | "
                f"SESSION={sid:016X} | {chuoi_rssi_snr(rssi, snr)}"
            )

            # SU có thể nghe ké DU->rBS READY. Cho SU xả packet + re-arm RX.
            time.sleep(SESSION_READY_FORWARD_GUARD)

            print(
                f"[THỜI GIAN][rBS] GUARD trước khi chuyển READY sang SU = "
                f"{SESSION_READY_FORWARD_GUARD * 1000:.1f} ms"
            )

            gui_session_ready_cho_su(
                rfm9x,
                sid,
            )
            ket_qua["so_lan_ready_toi_su"] += 1

            print(
                f"[CHẨN ĐOÁN S3] rBS -> SU: ĐÃ PHÁT SESSION_READY | "
                f"SESSION={sid:016X}"
            )

            ket_qua["ok"] = True
            return ket_qua

        print(
            f"[rBS BẮT TAY] CHƯA NHẬN SESSION_READY từ DU | "
            f"LẦN={attempt}/{SESSION_MAX_ATTEMPTS}"
        )

    gui_session_fail_cho_su(
        rfm9x,
        session_id,
    )

    print(
        "[KẾT LUẬN CHẨN ĐOÁN] BẮT TAY SESSION THẤT BẠI: "
        "rBS không nhận được SESSION_READY sau 3 lần."
    )
    print(
        "[KẾT LUẬN CHẨN ĐOÁN] CHƯA XÁC ĐỊNH chính xác: "
        "có thể mất SESSION_START ở rBS->DU hoặc mất SESSION_READY ở DU->rBS."
    )

    return ket_qua


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
        f"[rBS GỬI] PLAY_STARTED -> SU | "
        f"SESSION={session_id:016X}"
    )


def parse_user_response_du(packet_bytes):

    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)

    if len(p) != SIZE_USER_HMI:
        return None

    if (
        p[0] != ID_TRAM_RBS
        or p[1] != ID_TRAM_DU
        or p[2] != TYPE_USER_RESPONSE
        or p[3] not in (USER_RESPONSE_ACK, USER_RESPONSE_NACK)
    ):
        return None

    sid = int.from_bytes(p[4:12], byteorder="big", signed=False)
    if sid == 0:
        return None

    return {"session_id": sid, "code": p[3]}


def gui_user_response_cho_su(rfm9x, session_id, code):

    rfm9x.send(
        int(session_id).to_bytes(8, byteorder="big", signed=False),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_USER_RESPONSE,
        flags=code,
    )

    print(
        f"[rBS PHẢN HỒI] {'ACK TỰ ĐỘNG' if code == USER_RESPONSE_ACK else 'NACK THỦ CÔNG'} "
        f"-> SU | SESSION={session_id:016X}"
    )


def parse_user_confirm_su(packet_bytes):

    if packet_bytes is None:
        return None

    p = bytes(packet_bytes)

    if len(p) != SIZE_USER_HMI:
        return None

    if (
        p[0] != ID_TRAM_RBS
        or p[1] != ID_TRAM_SU
        or p[2] != TYPE_USER_CONFIRM
        or p[3] not in (USER_RESPONSE_ACK, USER_RESPONSE_NACK)
    ):
        return None

    sid = int.from_bytes(p[4:12], byteorder="big", signed=False)
    if sid == 0:
        return None

    return {"session_id": sid, "code": p[3]}


def gui_user_confirm_cho_du(rfm9x, session_id, code):

    rfm9x.send(
        int(session_id).to_bytes(8, byteorder="big", signed=False),
        destination=ID_TRAM_DU,
        node=ID_TRAM_RBS,
        identifier=TYPE_USER_CONFIRM,
        flags=code,
    )

    print(
        f"[rBS XÁC NHẬN] SU đã nhận {'ACK' if code == USER_RESPONSE_ACK else 'NACK'} -> DU | "
        f"SESSION={session_id:016X}"
    )


def gui_end_audio_cho_du(rfm9x, expected_session_id=None, diag_session=None):
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

    print("[rBS GỬI] END_AUDIO #1 -> DU | MỞ CỬA SỔ CHỜ PLAY_STARTED")

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

        rssi_play, snr_play = lay_rssi_snr(rfm9x)

        if diag_session is not None:
            cap_nhat_thong_ke_lien_ket(
                diag_session["link_du_rbs"],
                rssi_play,
                snr_play,
                "PLAY_STARTED",
            )

        print(
            f"[rBS NHẬN] PLAY_STARTED từ DU | "
            f"SESSION={sid:016X} | {chuoi_rssi_snr(rssi_play, snr_play)}"
        )

        # SU cung nghe thay packet DU->rBS. Cho SU doc/xả packet do
        # va re-arm RX continuous truoc khi rBS forward report.
        time.sleep(PLAY_REPORT_FORWARD_GUARD)

        print(
            f"[THỜI GIAN][rBS] GUARD trước khi chuyển PLAY_STARTED sang SU = "
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
            "[rBS CẢNH BÁO] HẾT THỜI GIAN CHỜ PLAY_STARTED -> SU sẽ hiện E2E ---"
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

    print("[rBS GỬI] END_AUDIO #2/#3 -> DU")


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
        f"[rBS] BẮT ĐẦU PHA 2 | CHUYỂN TIẾP 1 GÓI {info['kind'].upper()}"
    )
    print("----------------------------------------------------")

    if t_rx_us is not None:
        print(
            f"[THỜI GIAN][rBS] TỪ LÚC NHẬN GÓI -> BẮT ĐẦU PHA 2 = "
            f"{t_start - t_rx_us} us"
        )

    if session_da_gui:
        t0 = now_us()
        time.sleep(PHASE2_RX_GUARD)
        t1 = now_us()

        print(
            f"[THỜI GIAN][rBS] PHASE2_RX_GUARD = "
            f"{(t1 - t0) / 1000:.3f} ms"
        )

    # Nhánh dự phòng cũ. Với SESSION_READY bình thường, DU đã READY trước DATA.
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
            f"[rBS GỬI] SESSION_START dự phòng -> DU | "
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
            f"[rBS GỬI DATA] VOICE -> DU | "
            f"SEQ={info['seq']} | "
            f"FRAME={info['frames']} | "
            f"LAST={1 if info['last_audio'] else 0}"
        )
    else:
        print(
            f"[rBS GỬI DATA] FEC -> DU | "
            f"GROUP_START={info['group_start']} | "
            f"DATA={info['data_count']} | "
            f"HAS_LAST={1 if info['has_last'] else 0}"
        )

    print(
        f"[THỜI GIAN][rBS GỬI] THỜI GIAN PHÁT = "
        f"{t_tx1 - t_tx0} us "
        f"({(t_tx1 - t_tx0) / 1000:.3f} ms)"
    )

    # Cho SU xả packet relay 180B nghe ké và quay lại RX.
    tg0 = now_us()
    time.sleep(READY_GUARD)
    tg1 = now_us()

    print(
        f"[THỜI GIAN][rBS] READY_GUARD = "
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
        f"[THỜI GIAN][rBS] THỜI GIAN PHÁT ACK = "
        f"{ta1 - ta0} us "
        f"({(ta1 - ta0) / 1000:.3f} ms)"
    )

    print("[rBS] KẾT THÚC PHA 2")
    print("[rBS] QUAY LẠI PHA 1 -> NGHE SU")
    print("----------------------------------------------------")
    print()

    return session_da_gui, ta1


def reset_session_state():
    return (
        None,   # session_id
        None,   # goi_session
        False,  # session_da_gui / DU READY
        False,  # session_setup_failed
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
    print(" SESSION = START -> DU READY -> MỚI CHO PHÉP VOICE")
    print(" HMI = DU ACK TỰ ĐỘNG / NACK THỦ CÔNG -> SU -> XÁC NHẬN -> DU")
    print(" LOG LINK = RSSI/SNR TB-MIN-MAX + SỐ GÓI + ARQ RETRY THEO SESSION")
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
            "[rBS LỖI] Khởi tạo LoRa thất bại:",
            error,
        )
        return

    print("[rBS] Khởi tạo LoRa THÀNH CÔNG")
    print("[rBS] PHA 1 -> ĐANG NGHE SU")
    print()

    gps_manager = QuanLyGPSRBS()

    (
        session_id_hien_tai,
        goi_session,
        session_da_gui,
        session_setup_failed,
        da_relay_data,
        deadline_end_audio,
        seen_keys,
    ) = reset_session_state()

    t_ack_end_us = None
    diag_session = None
    diag_hoan_tat_gan_nhat = None

    while True:

        t_rx_call_us = now_us()

        if t_ack_end_us is not None:
            print(
                f"[THỜI GIAN][rBS] TỪ ACK CUỐI -> GỌI HÀM NHẬN = "
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

            rssi_goi, snr_goi = lay_rssi_snr(rfm9x)

            # GPS REPORT SU/DU -> rBS. GPS la telemetry, khong gate VOICE.
            gps_report = parse_gps_report(packet)
            if gps_report is not None:
                gps_manager.cap_nhat(gps_report, rssi_goi, snr_goi)

                diag_gps = None
                if diag_session is not None and diag_session["session_id"] == gps_report["session_id"]:
                    diag_gps = diag_session
                elif diag_hoan_tat_gan_nhat is not None and diag_hoan_tat_gan_nhat["session_id"] == gps_report["session_id"]:
                    diag_gps = diag_hoan_tat_gan_nhat

                if diag_gps is not None:
                    link_key = "link_su_rbs" if gps_report["src"] == GPS_ID_SU else "link_du_rbs"
                    cap_nhat_thong_ke_lien_ket(
                        diag_gps[link_key],
                        rssi_goi,
                        snr_goi,
                        "GPS_REPORT",
                    )

                ten = "SU" if gps_report["src"] == GPS_ID_SU else "DU"
                print(
                    f"[rBS GPS] NHẬN GPS_REPORT từ {ten} | SESSION={gps_report['session_id']:016X} | "
                    f"VALID={1 if gps_report['gps_valid'] else 0} | "
                    f"LAT={gps_report['latitude']:.7f} | LON={gps_report['longitude']:.7f} | "
                    f"ALT={gps_report['altitude_m']:.1f}m | SPEED={gps_report['speed_mps']:.2f}m/s | "
                    f"SAT={gps_report['satellites']} | HDOP={gps_report['hdop']:.2f} | "
                    f"AGE={gps_report['fix_age_ms']} ms | {chuoi_rssi_snr(rssi_goi, snr_goi)}"
                )
                continue

            # HMI: DU -> rBS -> SU
            user_response = parse_user_response_du(packet)
            if user_response is not None:
                diag_hmi = None
                if (
                    diag_session is not None
                    and diag_session["session_id"] == user_response["session_id"]
                ):
                    diag_hmi = diag_session
                elif (
                    diag_hoan_tat_gan_nhat is not None
                    and diag_hoan_tat_gan_nhat["session_id"] == user_response["session_id"]
                ):
                    diag_hmi = diag_hoan_tat_gan_nhat

                if diag_hmi is not None:
                    cap_nhat_thong_ke_lien_ket(
                        diag_hmi["link_du_rbs"],
                        rssi_goi,
                        snr_goi,
                        "ACK_TỰ_ĐỘNG" if user_response["code"] == USER_RESPONSE_ACK else "NACK_THỦ_CÔNG",
                    )
                    diag_hmi["hmi_ket_qua"] = (
                        "ACK" if user_response["code"] == USER_RESPONSE_ACK else "NACK"
                    )

                print(
                    f"[rBS NHẬN PHẢN HỒI] "
                    f"{'ACK TỰ ĐỘNG' if user_response['code'] == USER_RESPONSE_ACK else 'NACK THỦ CÔNG'} "
                    f"từ DU | SESSION={user_response['session_id']:016X} | "
                    f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                )
                time.sleep(USER_HMI_FORWARD_GUARD)
                gui_user_response_cho_su(
                    rfm9x,
                    user_response["session_id"],
                    user_response["code"],
                )
                continue

            # HMI: SU confirm -> rBS -> DU
            user_confirm = parse_user_confirm_su(packet)
            if user_confirm is not None:
                diag_hmi = None
                if (
                    diag_session is not None
                    and diag_session["session_id"] == user_confirm["session_id"]
                ):
                    diag_hmi = diag_session
                elif (
                    diag_hoan_tat_gan_nhat is not None
                    and diag_hoan_tat_gan_nhat["session_id"] == user_confirm["session_id"]
                ):
                    diag_hmi = diag_hoan_tat_gan_nhat

                if diag_hmi is not None:
                    cap_nhat_thong_ke_lien_ket(
                        diag_hmi["link_su_rbs"],
                        rssi_goi,
                        snr_goi,
                        "USER_CONFIRM",
                    )

                print(
                    f"[rBS NHẬN XÁC NHẬN] SU đã nhận "
                    f"{'ACK' if user_confirm['code'] == USER_RESPONSE_ACK else 'NACK'} | "
                    f"SESSION={user_confirm['session_id']:016X} | "
                    f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                )
                time.sleep(USER_HMI_FORWARD_GUARD)
                gui_user_confirm_cho_du(
                    rfm9x,
                    user_confirm["session_id"],
                    user_confirm["code"],
                )

                if diag_hmi is not None:
                    in_tong_ket_lien_ket(
                        diag_hmi,
                        "PHẢN HỒI DU ĐÃ ĐƯỢC SU XÁC NHẬN",
                        gps_manager,
                    )
                continue

            info = parse_packet_su(
                packet
            )

            if info is None:
                continue

            # ----------------------------------------------------
            # AUDIO_END
            # ----------------------------------------------------
            if info["kind"] == "audio_end":

                if diag_session is not None:
                    cap_nhat_thong_ke_lien_ket(
                        diag_session["link_su_rbs"],
                        rssi_goi,
                        snr_goi,
                        "AUDIO_END",
                    )

                print(
                    f"[rBS NHẬN] AUDIO_END 5B từ SU | "
                    f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                )

                if da_relay_data:
                    gui_end_audio_cho_du(
                        rfm9x,
                        session_id_hien_tai,
                        diag_session,
                    )

                    print("[rBS] PHIÊN THOẠI KẾT THÚC BẰNG AUDIO_END")

                if diag_session is not None:
                    in_tong_ket_lien_ket(diag_session, "AUDIO_END", gps_manager)
                    diag_hoan_tat_gan_nhat = diag_session

                (
                    session_id_hien_tai,
                    goi_session,
                    session_da_gui,
                    session_setup_failed,
                    da_relay_data,
                    deadline_end_audio,
                    seen_keys,
                ) = reset_session_state()
                diag_session = None

                print("[rBS] CHỜ PHIÊN MỚI...")
                print()
                continue

            # ----------------------------------------------------
            # SESSION START -> SESSION_READY HANDSHAKE
            # ----------------------------------------------------
            if info["kind"] == "session":

                session_moi = info["session_id"]

                if session_id_hien_tai != session_moi:
                    if da_relay_data:
                        gui_end_audio_cho_du(
                            rfm9x,
                            session_id_hien_tai,
                            diag_session,
                        )
                        if diag_session is not None:
                            in_tong_ket_lien_ket(diag_session, "SESSION_MỚI ĐẾN TRƯỚC AUDIO_END", gps_manager)
                            diag_hoan_tat_gan_nhat = diag_session

                    session_id_hien_tai = session_moi
                    goi_session = info["raw"]
                    session_da_gui = False
                    session_setup_failed = False
                    da_relay_data = False
                    deadline_end_audio = None
                    seen_keys.clear()

                    diag_session = tao_chan_doan_session(session_moi)
                    cap_nhat_thong_ke_lien_ket(
                        diag_session["link_su_rbs"],
                        rssi_goi,
                        snr_goi,
                        "SESSION_START",
                    )
                    print(
                        f"[CHẨN ĐOÁN S0] SU -> rBS: NHẬN SESSION_START = OK | "
                        f"SESSION={session_moi:016X} | "
                        f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                    )

                else:
                    goi_session = info["raw"]

                    if diag_session is None:
                        diag_session = tao_chan_doan_session(session_moi)
                    cap_nhat_thong_ke_lien_ket(
                        diag_session["link_su_rbs"],
                        rssi_goi,
                        snr_goi,
                        "SESSION_START_LẶP",
                    )
                    diag_session["so_lan_su_gui_lai_start"] += 1

                    print(
                        f"[rBS NHẬN] SU gửi lại SESSION_START | "
                        f"SESSION={session_moi:016X} | "
                        f"LẦN_LẶP={diag_session['so_lan_su_gui_lai_start']} | "
                        f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                    )

                # Neu READY da thanh cong nhung SU bi mat packet READY,
                # SU se gui lai cung SESSION_START. Khong bat tay lai DU;
                # chi forward READY lai cho SU.
                if session_da_gui:
                    print(
                        "[CHẨN ĐOÁN] DU đã READY từ trước nhưng SU lại gửi SESSION_START."
                    )
                    print(
                        "[CHẨN ĐOÁN] Khả năng cao gói SESSION_READY rBS->SU trước đó đã bị mất; "
                        "rBS sẽ gửi lại READY cho SU."
                    )
                    gui_session_ready_cho_su(
                        rfm9x,
                        session_id_hien_tai,
                    )
                    if diag_session is not None:
                        diag_session["so_lan_ready_toi_su"] += 1
                        in_tom_tat_chan_doan(diag_session)
                    continue

                # Neu da fail 3 lan, duplicate tu SU chi nhan FAIL lai,
                # khong khoi dong them 3 retry moi.
                if session_setup_failed:
                    print(
                        "[rBS BẮT TAY] Session này đã thất bại trước đó -> gửi lại SESSION_FAIL cho SU."
                    )
                    gui_session_fail_cho_su(
                        rfm9x,
                        session_id_hien_tai,
                    )
                    if diag_session is not None:
                        in_tom_tat_chan_doan(diag_session)
                    continue

                ket_qua_bat_tay = thiet_lap_session_voi_du(
                    rfm9x,
                    goi_session,
                    session_id_hien_tai,
                    diag_session,
                    gps_manager,
                )

                if diag_session is None:
                    diag_session = tao_chan_doan_session(session_id_hien_tai)
                diag_session["so_lan_start_toi_du"] += ket_qua_bat_tay["so_lan_start_toi_du"]
                diag_session["da_nhan_ready_tu_du"] = (
                    diag_session["da_nhan_ready_tu_du"]
                    or ket_qua_bat_tay["da_nhan_ready_tu_du"]
                )
                diag_session["so_lan_ready_toi_su"] += ket_qua_bat_tay["so_lan_ready_toi_su"]

                if ket_qua_bat_tay["ok"]:
                    session_da_gui = True
                    session_setup_failed = False
                    print(
                        f"[rBS BẮT TAY] THÀNH CÔNG | SESSION={session_id_hien_tai:016X}"
                    )
                    print(
                        "[CHẨN ĐOÁN] Đường DU -> rBS = OK vì rBS đã nhận SESSION_READY."
                    )
                    print(
                        "[CHẨN ĐOÁN] Đường rBS -> SU chưa được xác nhận tuyệt đối; "
                        "sẽ xác nhận gián tiếp khi rBS nhận VOICE đầu tiên từ SU."
                    )
                else:
                    session_da_gui = False
                    session_setup_failed = True
                    print(
                        f"[rBS BẮT TAY] THẤT BẠI | SESSION={session_id_hien_tai:016X}"
                    )

                in_tom_tat_chan_doan(diag_session)
                if not ket_qua_bat_tay["ok"]:
                    in_tong_ket_lien_ket(diag_session, "SESSION_FAIL", gps_manager)
                continue

            # ----------------------------------------------------
            # VOICE / FEC
            # ----------------------------------------------------
            if info["kind"] in ("voice", "fec"):

                if session_id_hien_tai is None:
                    print(
                        "[rBS BỎ GÓI] DATA đến khi chưa có SESSION"
                    )
                    continue

                if not session_da_gui:
                    print(
                        "[rBS BỎ GÓI] DATA đến khi DU chưa SESSION_READY"
                    )
                    continue

                if diag_session is not None:
                    cap_nhat_thong_ke_lien_ket(
                        diag_session["link_su_rbs"],
                        rssi_goi,
                        snr_goi,
                        "VOICE" if info["kind"] == "voice" else "FEC",
                    )

                if info["kind"] == "voice":
                    key = (
                        TYPE_VOICE,
                        info["seq"],
                    )

                    print(
                        f"[rBS NHẬN DATA] VOICE từ SU | "
                        f"SEQ={info['seq']} | "
                        f"FRAME={info['frames']} | "
                        f"LAST={1 if info['last_audio'] else 0} | "
                        f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
                    )

                    if diag_session is not None and not diag_session["da_nhan_voice_dau_tien"]:
                        diag_session["da_nhan_voice_dau_tien"] = True
                        print(
                            f"[CHẨN ĐOÁN S4] SU -> rBS: NHẬN VOICE ĐẦU TIÊN = OK | "
                            f"SEQ={info['seq']}"
                        )
                        print(
                            "[KẾT LUẬN CHẨN ĐOÁN] BẮT TAY SESSION THÀNH CÔNG."
                        )
                        print(
                            "[KẾT LUẬN CHẨN ĐOÁN] DU -> rBS = OK "
                            "(rBS đã nhận SESSION_READY)."
                        )
                        print(
                            "[KẾT LUẬN CHẨN ĐOÁN] rBS -> SU = OK GIÁN TIẾP "
                            "(SU chỉ gửi VOICE sau khi nhận SESSION_READY)."
                        )
                        in_tom_tat_chan_doan(diag_session)
                else:
                    key = (
                        TYPE_FEC,
                        info["group_start"],
                    )

                    print(
                        f"[rBS NHẬN DATA] FEC từ SU | "
                        f"GROUP_START={info['group_start']} | "
                        f"DATA={info['data_count']} | "
                        f"HAS_LAST={1 if info['has_last'] else 0} | "
                        f"FINAL_FRAME={info['final_frames']} | "
                        f"{chuoi_rssi_snr(rssi_goi, snr_goi)}"
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

                    if diag_session is not None:
                        diag_session["so_goi_trung_arq"] += 1

                    print(
                        f"[rBS ARQ] GÓI TRÙNG -> KHÔNG CHUYỂN TIẾP LẠI | "
                        f"LOẠI=0x{info['ack_kind']:02X} | "
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
                        f"[THỜI GIAN][rBS] THỜI GIAN ACK CHO GÓI TRÙNG = "
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
                "[rBS DỰ PHÒNG] HẾT THỜI GIAN CHỜ AUDIO_END -> GỬI END SANG DU"
            )

            if da_relay_data:
                gui_end_audio_cho_du(
                    rfm9x,
                    session_id_hien_tai,
                    diag_session,
                )

            if diag_session is not None:
                in_tong_ket_lien_ket(diag_session, "FALLBACK HẾT THỜI GIAN AUDIO_END", gps_manager)
                diag_hoan_tat_gan_nhat = diag_session

            (
                session_id_hien_tai,
                goi_session,
                session_da_gui,
                session_setup_failed,
                da_relay_data,
                deadline_end_audio,
                seen_keys,
            ) = reset_session_state()
            diag_session = None

            print("[rBS] CHỜ PHIÊN MỚI...")
            print()


if __name__ == "__main__":
    main()
