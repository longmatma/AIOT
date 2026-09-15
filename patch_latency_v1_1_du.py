from pathlib import Path
from datetime import datetime
import shutil

ROOT = Path(__file__).resolve().parent
MAIN = ROOT / "DU" / "src" / "main.cpp"

if not MAIN.exists():
    raise SystemExit(f"[FAIL] Khong tim thay: {MAIN}")

t = MAIN.read_text(encoding="utf-8")

if "[DU CRYPTO] FIXED AES-128-GCM | SESSION KEY THEO SESSION_ID" not in t:
    raise SystemExit(
        "[FAIL] DU chua o baseline FIXED AES-128-GCM. Dung lai."
    )

if "Gui_BaoCao_BaoMat_DangCho" in t:
    raise SystemExit(
        "[FAIL] DU van con security telemetry 0x1A. Hay hoan tat B2 truoc."
    )

if "DU_Latency_SessionBusy" in t:
    raise SystemExit("[FAIL] LATENCY V1.1 co ve da duoc patch truoc do.")

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup = ROOT / f"backup_before_latency_v1_1_{stamp}"
backup.mkdir(parents=True, exist_ok=False)

dst = backup / MAIN.relative_to(ROOT)
dst.parent.mkdir(parents=True, exist_ok=True)
shutil.copy2(MAIN, dst)

print(f"[BACKUP] {backup}")


def rep(text, old, new, label):
    n = text.count(old)
    if n != 1:
        raise RuntimeError(
            f"[STOP] {label}: can dung 1 vi tri, tim thay {n}"
        )
    print(f"[PATCH] {label}")
    return text.replace(old, new, 1)


anchor = '''// DU chỉ bắt đầu lấy PCM ra khỏi Audio_Buffer sau khi
// nhận END_AUDIO từ rBS.
volatile bool cho_phep_phat_audio = false;
'''

insert = '''// DU chỉ bắt đầu lấy PCM ra khỏi Audio_Buffer sau khi
// nhận END_AUDIO từ rBS.
volatile bool cho_phep_phat_audio = false;


// =====================================================
// LATENCY OPTIMIZATION V1.1
//
// 1) SESSION_READY duoc uu tien TX ngay khi DU nhan SESSION_START.
// 2) GPS_PHIEN khong chen vao cua so handshake/VOICE/FEC.
//    GPS_PHIEN van duoc GIU, nhung dua sang pending va gui khi radio ranh.
// 3) GPS dinh ky + report kenh SU-DU bi tam khoa trong luc DU dang
//    nhan mot session thoai.
//
// Timeout 30s chi la fail-safe: neu mat END_AUDIO, telemetry tu mo lai.
// =====================================================
static portMUX_TYPE DU_Latency_Mux =
    portMUX_INITIALIZER_UNLOCKED;

static volatile bool DU_Latency_SessionBusy = false;
static volatile uint32_t DU_Latency_SessionStartMs = 0;

static bool DU_Latency_GPSPhienPending = false;
static uint64_t DU_Latency_GPSPhienId = 0;

static void DU_Latency_BatDauSession(uint64_t session_id)
{
    const uint32_t now_ms = millis();

    portENTER_CRITICAL(&DU_Latency_Mux);

    DU_Latency_SessionBusy = true;
    DU_Latency_SessionStartMs = now_ms;

    DU_Latency_GPSPhienPending = true;
    DU_Latency_GPSPhienId = session_id;

    portEXIT_CRITICAL(&DU_Latency_Mux);
}

static void DU_Latency_KetThucNhanSession()
{
    portENTER_CRITICAL(&DU_Latency_Mux);
    DU_Latency_SessionBusy = false;
    portEXIT_CRITICAL(&DU_Latency_Mux);
}

static bool DU_Latency_DangNhanSession()
{
    bool busy = false;
    uint32_t start_ms = 0;

    portENTER_CRITICAL(&DU_Latency_Mux);
    busy = DU_Latency_SessionBusy;
    start_ms = DU_Latency_SessionStartMs;
    portEXIT_CRITICAL(&DU_Latency_Mux);

    if (
        busy
        &&
        (uint32_t)(millis() - start_ms) > 30000UL
    )
    {
        DU_Latency_KetThucNhanSession();

        Serial.println(
            "[DU LATENCY] SESSION RX TIMEOUT -> MO LAI TELEMETRY"
        );

        return false;
    }

    return busy;
}

static bool DU_Latency_LayGPSPhienPending(uint64_t &session_id)
{
    bool pending = false;

    portENTER_CRITICAL(&DU_Latency_Mux);

    pending = DU_Latency_GPSPhienPending;

    if (pending)
        session_id = DU_Latency_GPSPhienId;

    portEXIT_CRITICAL(&DU_Latency_Mux);

    return pending;
}

static void DU_Latency_XoaGPSPhienPending(uint64_t session_id)
{
    portENTER_CRITICAL(&DU_Latency_Mux);

    if (
        DU_Latency_GPSPhienPending
        &&
        DU_Latency_GPSPhienId == session_id
    )
    {
        DU_Latency_GPSPhienPending = false;
    }

    portEXIT_CRITICAL(&DU_Latency_Mux);
}
'''

t = rep(t, anchor, insert, "add latency/session telemetry state")

old = '''                // END da toi -> module HMI ket thuc trang thai RX session.
                HMI_DU_Bao_END_Audio();

                // Mo gate TRUOC khi ve OLED. Audio task o core0 co the
'''

new = '''                // END da toi -> khong con nhan VOICE/FEC cua session nay.
                // Tu day playback se tiep tuc khoa telemetry bang
                // cho_phep_phat_audio.
                DU_Latency_KetThucNhanSession();

                // END da toi -> module HMI ket thuc trang thai RX session.
                HMI_DU_Bao_END_Audio();

                // Mo gate TRUOC khi ve OLED. Audio task o core0 co the
'''

t = rep(t, old, new, "END_AUDIO release receive-session lock")

old = '''                HMI_DU_TamDung_Beacon(1500UL);

                Serial.printf(
                    "[DU] SESSION LAP LAI = %016llX -> GUI LAI SESSION_READY\\n",
                    (unsigned long long)session_moi
                );

                DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();
                Gui_GPS_REPORT_DU(session_moi, du_lieu_gps_du);
                Gui_SESSION_READY_RBS(session_moi);
                continue;
'''

new = '''                HMI_DU_TamDung_Beacon(1500UL);

                DU_Latency_BatDauSession(session_moi);

                Serial.printf(
                    "[DU] SESSION LAP LAI = %016llX -> GUI NGAY SESSION_READY\\n",
                    (unsigned long long)session_moi
                );

                // Latency V1.1: READY la control gate cua voice, nen uu tien.
                // GPS_PHIEN da duoc dua vao pending trong DU_Latency_BatDauSession().
                Gui_SESSION_READY_RBS(session_moi);
                continue;
'''

t = rep(t, old, new, "duplicate SESSION_READY before GPS")

old = '''            // GPS packet rieng: gui snapshot truoc READY de rBS thu duoc
            // trong cua so bat tay. GPS khong gate session.
            DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();
            In_TrangThai_GPS_DU(du_lieu_gps_du);
            Gui_GPS_REPORT_DU(session_id_hien_tai, du_lieu_gps_du);

            // May DU tu dong xac nhan, nguoi dung KHONG can bam nut.
            Gui_SESSION_READY_RBS(session_id_hien_tai);
'''

new = '''            // Latency V1.1:
            // SESSION_READY la control gate cho voice -> gui NGAY.
            // GPS_PHIEN van giu nguyen chuc nang nhung chuyen sang pending,
            // chi TX sau khi cua so SESSION/VOICE/FEC da ket thuc.
            DU_Latency_BatDauSession(session_id_hien_tai);

            Gui_SESSION_READY_RBS(session_id_hien_tai);
'''

t = rep(t, old, new, "new SESSION_READY before/defer GPS")

old = '''        bool radio_dang_ban = HMI_DU_Radio_Dang_Ban() || cho_phep_phat_audio;
        if (!radio_dang_ban)
            Gui_BaoCao_Kenh_SU_DU_DangCho();
'''

new = '''        bool radio_dang_ban =
            HMI_DU_Radio_Dang_Ban()
            || cho_phep_phat_audio
            || DU_Latency_DangNhanSession();

        if (!radio_dang_ban)
            Gui_BaoCao_Kenh_SU_DU_DangCho();
'''

t = rep(t, old, new, "lock SU-DU channel report during voice session")

old = '''        bool radio_dang_ban =
            HMI_DU_Radio_Dang_Ban()
            || cho_phep_phat_audio;

        if (!radio_dang_ban &&
            (int32_t)(bay_gio_ms - moc_gui_tiep_theo_ms) >= 0)
        {
            DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();
            In_TrangThai_GPS_DU(du_lieu_gps_du);

            // V10: chi tang STT SAU KHI TX thanh cong.
'''

new = '''        bool radio_dang_ban =
            HMI_DU_Radio_Dang_Ban()
            || cho_phep_phat_audio
            || DU_Latency_DangNhanSession();

        // GPS_PHIEN van duoc bao toan, nhung khong duoc chen vao
        // handshake/VOICE/FEC. Khi radio ranh, gui no truoc GPS dinh ky.
        uint64_t gps_phien_pending_id = 0;

        if (
            !radio_dang_ban
            &&
            DU_Latency_LayGPSPhienPending(gps_phien_pending_id)
        )
        {
            DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();
            In_TrangThai_GPS_DU(du_lieu_gps_du);

            if (
                Gui_GPS_REPORT_DU(
                    gps_phien_pending_id,
                    du_lieu_gps_du
                )
            )
            {
                DU_Latency_XoaGPSPhienPending(
                    gps_phien_pending_id
                );
            }
        }
        else if (
            !radio_dang_ban
            &&
            (int32_t)(bay_gio_ms - moc_gui_tiep_theo_ms) >= 0
        )
        {
            DuLieuGPS_DU du_lieu_gps_du = Lay_DuLieu_GPS_DU();
            In_TrangThai_GPS_DU(du_lieu_gps_du);

            // V10: chi tang STT SAU KHI TX thanh cong.
'''

t = rep(t, old, new, "defer session GPS + lock periodic GPS during voice")

MAIN.write_text(t, encoding="utf-8")

final = MAIN.read_text(encoding="utf-8")

for marker in (
    "DU_Latency_BatDauSession",
    "DU_Latency_DangNhanSession",
    "GUI NGAY SESSION_READY",
    "GPS_PHIEN van duoc bao toan",
    "[DU CRYPTO] FIXED AES-128-GCM",
):
    if marker not in final:
        raise RuntimeError(
            f"[STOP] Thieu marker sau patch: {marker}"
        )

bad = '''Gui_GPS_REPORT_DU(session_id_hien_tai, du_lieu_gps_du);

            // May DU tu dong xac nhan, nguoi dung KHONG can bam nut.
            Gui_SESSION_READY_RBS(session_id_hien_tai);'''

if bad in final:
    raise RuntimeError("[STOP] Van con GPS_PHIEN truoc SESSION_READY")

print()
print("[DONE] LATENCY OPTIMIZATION V1.1 - DU")
print("[1] SESSION_READY duoc gui ngay, khong doi GPS lay quyen voice")
print("[2] GPS_PHIEN duoc defer, KHONG bi xoa")
print("[3] GPS dinh ky + report kenh bi khoa khi dang SESSION/VOICE/FEC")
print("[4] Playback/HMI van giu co che khoa radio cu")
print("[5] SU khong can sua: telemetry SU da chi gui khi NGHI_NGOI")
print("[NEXT] Build DU. Neu SUCCESS moi Upload DU va test PTT_RELEASE -> DU_PLAY.")
