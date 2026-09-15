from pathlib import Path
from datetime import datetime
import shutil

ROOT = Path(__file__).resolve().parent

SU_MAIN = ROOT / "SU" / "src" / "main.cpp"
DU_MAIN = ROOT / "DU" / "src" / "main.cpp"
DU_CPP = ROOT / "DU" / "src" / "nhan_lora.cpp"
DU_H = ROOT / "DU" / "src" / "nhan_lora.h"

for p in (SU_MAIN, DU_MAIN, DU_CPP, DU_H):
    if not p.exists():
        raise SystemExit(f"[FAIL] Khong tim thay: {p}")

su = SU_MAIN.read_text(encoding="utf-8")
du = DU_MAIN.read_text(encoding="utf-8")
du_cpp = DU_CPP.read_text(encoding="utf-8")
du_h = DU_H.read_text(encoding="utf-8")

if "TELEMETRY PRE/POST V1" in su or "TELEMETRY PRE/POST V1" in du:
    raise SystemExit("[FAIL] TELEMETRY PRE/POST V1 co ve da duoc patch.")

for marker in (
    "DU_Latency_SessionBusy",
    "DU_Latency_GPSPhienPending",
    "DU_Latency_BatDauSession",
    "DU_Latency_DangNhanSession",
):
    if marker not in du:
        raise SystemExit(
            f"[FAIL] DU chua co Latency V1.1: thieu {marker!r}. "
            "Hay chay patch_latency_v1_1_du.py truoc."
        )

if "[DU CRYPTO] FIXED AES-128-GCM | SESSION KEY THEO SESSION_ID" not in du:
    raise SystemExit("[FAIL] DU chua o baseline Fixed AES-128-GCM.")

for ptxt, name in (
    (du, "DU/main.cpp"),
    (du_cpp, "DU/nhan_lora.cpp"),
    (du_h, "DU/nhan_lora.h"),
):
    if "Gui_BaoCao_BaoMat_DangCho" in ptxt or "TYPE_BAO_CAO_BAO_MAT" in ptxt:
        raise SystemExit(
            f"[FAIL] {name} van con security telemetry 0x1A. "
            "Hay hoan tat B2 cleanup truoc."
        )

for marker in (
    "XuLy_BaoCao_ViTri_DinhKy_SU",
    "HMI_SU_Dang_Cho_PhanHoi",
    "moc_gui_vi_tri_su_tiep_theo_ms",
    "trang_thai =\n            DANG_GHI_AM;",
    "trang_thai =\n            NGHI_NGOI;",
):
    if marker not in su:
        raise SystemExit(
            f"[FAIL] SU/main.cpp sai baseline: thieu {marker!r}"
        )

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup = ROOT / f"backup_before_telemetry_pre_post_v1_{stamp}"
backup.mkdir(parents=True, exist_ok=False)

for p in (SU_MAIN, DU_MAIN, DU_CPP, DU_H):
    dst = backup / p.relative_to(ROOT)
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dst)

print(f"[BACKUP] {backup}")


def rep(text, old, new, label):
    n = text.count(old)
    if n != 1:
        raise RuntimeError(
            f"[STOP] {label}: can 1 vi tri, tim thay {n}. Khong sua."
        )
    print(f"[PATCH] {label}")
    return text.replace(old, new, 1)


# ============================================================
# A) SU
# ============================================================

old = '''uint32_t moc_gui_vi_tri_su_tiep_theo_ms = 0;
uint64_t so_thu_tu_bao_cao_vi_tri_su = 0;
'''

new = '''uint32_t moc_gui_vi_tri_su_tiep_theo_ms = 0;
uint64_t so_thu_tu_bao_cao_vi_tri_su = 0;


// ========================================================
// TELEMETRY PRE/POST V1
//
// PRE:
// - Khong phat them packet khi bam PTT.
// - Dung snapshot telemetry IDLE gan nhat da gui truoc do.
//
// TRONG PHIEN:
// - trang_thai != NGHI_NGOI -> GPS telemetry bi khoa.
//
// POST:
// - Khi phan voice da ket thuc, dat pending.
// - Neu HMI van dang cho ACK/NACK/confirm thi tiep tuc cho.
// - Ngay khi HMI ket thuc, gui 1 beacon GPS SU ngay lap tuc.
// - Beacon POST nay cung la mau de DU do RSSI/SNR SU->DU.
//
// Khong thay packet format, khong them delay vao SESSION/VOICE/FEC.
// ========================================================
static uint32_t su_telemetry_last_tx_ms = 0;
static bool su_telemetry_post_pending = false;
'''

su = rep(su, old, new, "SU add PRE/POST telemetry state")


old = '''    if (gui_thanh_cong)
        so_thu_tu_bao_cao_vi_tri_su = stt_du_kien;

    moc_gui_vi_tri_su_tiep_theo_ms = bay_gio_ms +
        (gui_thanh_cong ? CHU_KY_BAO_CAO_VI_TRI_SU_MS : 500UL);
'''

new = '''    if (gui_thanh_cong)
    {
        so_thu_tu_bao_cao_vi_tri_su = stt_du_kien;
        su_telemetry_last_tx_ms = bay_gio_ms;

        if (su_telemetry_post_pending)
        {
            su_telemetry_post_pending = false;

            Serial.printf(
                "[SU TELEMETRY] POST -> rBS | STT=%llu\\n",
                (unsigned long long)stt_du_kien
            );
        }
    }

    moc_gui_vi_tri_su_tiep_theo_ms = bay_gio_ms +
        (gui_thanh_cong ? CHU_KY_BAO_CAO_VI_TRI_SU_MS : 500UL);
'''

su = rep(su, old, new, "SU mark POST telemetry after successful idle TX")


old = '''        trang_thai =
            DANG_GHI_AM;

        // Cau moi -> module HMI xoa ACK/NACK/session cua cau truoc.
'''

new = '''        // TELEMETRY PRE/POST V1:
        // Khong TX telemetry moi tai thoi diem PTT.
        // Dung snapshot IDLE gan nhat lam PRE de khong chen delay vao voice.
        if (su_telemetry_last_tx_ms != 0)
        {
            Serial.printf(
                "[SU TELEMETRY] PRE SNAPSHOT | AGE=%u ms | STT=%llu | LOCK TRONG PHIEN\\n",
                (unsigned int)(millis() - su_telemetry_last_tx_ms),
                (unsigned long long)so_thu_tu_bao_cao_vi_tri_su
            );
        }
        else
        {
            Serial.println(
                "[SU TELEMETRY] PRE SNAPSHOT CHUA CO | LOCK TRONG PHIEN"
            );
        }

        trang_thai =
            DANG_GHI_AM;

        // Cau moi -> module HMI xoa ACK/NACK/session cua cau truoc.
'''

su = rep(su, old, new, "SU lock telemetry at PTT without extra TX")


old = '''        // =================================================
        // VỀ TRẠNG THÁI NGHỈ
        // =================================================

        trang_thai =
            NGHI_NGOI;
'''

new = '''        // =================================================
        // VỀ TRẠNG THÁI NGHỈ
        // =================================================

        // TELEMETRY PRE/POST V1:
        // Dat POST pending NGAY khi voice/session da ket thuc.
        // XuLy_BaoCao... van tu cho neu HMI dang cho phan hoi,
        // nen packet POST khong chen vao control cua session.
        su_telemetry_post_pending = true;
        moc_gui_vi_tri_su_tiep_theo_ms = millis();

        Serial.println(
            "[SU TELEMETRY] POST PENDING | CHO HMI/RADIO RANH"
        );

        trang_thai =
            NGHI_NGOI;
'''

su = rep(su, old, new, "SU schedule one POST telemetry after session")


# ============================================================
# B) DU nhan_lora
# ============================================================

old = '''bool Gui_BaoCao_Kenh_SU_DU_DangCho();

#endif
'''

new = '''bool Gui_BaoCao_Kenh_SU_DU_DangCho();

// TELEMETRY PRE/POST V1:
// Xoa mau PRE con pending khi voice session bat dau.
// Nhu vay report sau session chi dung beacon SU moi (POST).
void Xoa_BaoCao_Kenh_SU_DU_DangCho();

#endif
'''

du_h = rep(du_h, old, new, "DU header add clear-stale channel API")


marker = '''// =====================================================
// GUI SNAPSHOT KENH SU -> DU VE rBS
'''

if du_cpp.count(marker) != 1:
    raise RuntimeError(
        f"[STOP] DU nhan_lora.cpp marker channel sender: tim thay {du_cpp.count(marker)}"
    )

clear_func = '''// =====================================================
// TELEMETRY PRE/POST V1
// XOA MAU KENH PRE CON PENDING KHI SESSION BAT DAU.
//
// Khong TX gi ca. Chi xoa snapshot RAM cu de sau session
// DU cho beacon SU moi va bao cao dung mau POST.
// =====================================================
void Xoa_BaoCao_Kenh_SU_DU_DangCho()
{
    portENTER_CRITICAL(&KenhSUDU_Mux);
    mau_kenh_su_du.pending = false;
    portEXIT_CRITICAL(&KenhSUDU_Mux);
}


'''

du_cpp = du_cpp.replace(marker, clear_func + marker, 1)
print("[PATCH] DU add clear stale PRE channel snapshot")


# ============================================================
# C) DU main
# ============================================================

old = '''static bool DU_Latency_GPSPhienPending = false;
static uint64_t DU_Latency_GPSPhienId = 0;
'''

new = '''static bool DU_Latency_GPSPhienPending = false;
static uint64_t DU_Latency_GPSPhienId = 0;

// Sau session, cho mot beacon SU moi de tao report kenh POST.
static bool DU_Latency_PostChannelPending = false;
'''

du = rep(du, old, new, "DU add POST channel state")


old = '''    DU_Latency_GPSPhienPending = true;
    DU_Latency_GPSPhienId = session_id;

    portEXIT_CRITICAL(&DU_Latency_Mux);
}
'''

new = '''    // GPS_PHIEN nay se duoc gui SAU session, khong gui trong handshake.
    DU_Latency_GPSPhienPending = true;
    DU_Latency_GPSPhienId = session_id;

    // Channel POST phai den tu beacon SU moi sau session.
    DU_Latency_PostChannelPending = true;

    portEXIT_CRITICAL(&DU_Latency_Mux);

    // Bo mau PRE con pending trong RAM, KHONG TX.
    Xoa_BaoCao_Kenh_SU_DU_DangCho();

    Serial.println(
        "[DU TELEMETRY] LOCK TRONG PHIEN | PRE GIU O rBS | POST PENDING"
    );
}
'''

du = rep(du, old, new, "DU session start = telemetry lock + POST pending")


old = '''static void DU_Latency_XoaGPSPhienPending(uint64_t session_id)
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

new = '''static void DU_Latency_XoaGPSPhienPending(uint64_t session_id)
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

static bool DU_Latency_PostChannelDangCho()
{
    bool pending = false;

    portENTER_CRITICAL(&DU_Latency_Mux);
    pending = DU_Latency_PostChannelPending;
    portEXIT_CRITICAL(&DU_Latency_Mux);

    return pending;
}

static void DU_Latency_DanhDauPostChannelDaGui()
{
    portENTER_CRITICAL(&DU_Latency_Mux);
    DU_Latency_PostChannelPending = false;
    portEXIT_CRITICAL(&DU_Latency_Mux);
}
'''

du = rep(du, old, new, "DU add POST channel helpers")


old = '''        if (!radio_dang_ban)
            Gui_BaoCao_Kenh_SU_DU_DangCho();

        vTaskDelay(pdMS_TO_TICKS(25));
'''

new = '''        if (!radio_dang_ban)
        {
            bool gui_kenh_ok =
                Gui_BaoCao_Kenh_SU_DU_DangCho();

            if (
                gui_kenh_ok
                &&
                DU_Latency_PostChannelDangCho()
            )
            {
                DU_Latency_DanhDauPostChannelDaGui();

                Serial.println(
                    "[DU TELEMETRY] POST CHANNEL SU-DU -> rBS"
                );
            }
        }

        vTaskDelay(pdMS_TO_TICKS(25));
'''

du = rep(du, old, new, "DU label first fresh channel report as POST")


old = '''            if (
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
'''

new = '''            if (
                Gui_GPS_REPORT_DU(
                    gps_phien_pending_id,
                    du_lieu_gps_du
                )
            )
            {
                DU_Latency_XoaGPSPhienPending(
                    gps_phien_pending_id
                );

                // POST packet vua gui xong -> khong gui them GPS dinh ky
                // ngay lap tuc, tranh 2 telemetry packet lien nhau.
                moc_gui_tiep_theo_ms =
                    bay_gio_ms
                    +
                    CHU_KY_BAO_CAO_VI_TRI_DU_MS;

                Serial.printf(
                    "[DU TELEMETRY] POST GPS_PHIEN -> rBS | SESSION=%016llX\\n",
                    (unsigned long long)gps_phien_pending_id
                );
            }
'''

du = rep(du, old, new, "DU POST GPS and defer next periodic packet")


old_comment = '''// GPS_PHIEN van duoc bao toan, nhung khong duoc chen vao
        // handshake/VOICE/FEC. Khi radio ranh, gui no truoc GPS dinh ky.
'''
new_comment = '''// TELEMETRY PRE/POST V1:
        // PRE = periodic idle snapshot da co o rBS.
        // TRONG PHIEN = khong TX telemetry.
        // POST = GPS_PHIEN session-bound nay, chi gui khi radio/HMI/playback ranh.
'''
if old_comment in du:
    du = du.replace(old_comment, new_comment, 1)
    print("[PATCH] DU comment PRE/POST semantics")


SU_MAIN.write_text(su, encoding="utf-8")
DU_MAIN.write_text(du, encoding="utf-8")
DU_CPP.write_text(du_cpp, encoding="utf-8")
DU_H.write_text(du_h, encoding="utf-8")

checks = {
    "SU": SU_MAIN.read_text(encoding="utf-8"),
    "DU": DU_MAIN.read_text(encoding="utf-8"),
    "DU_CPP": DU_CPP.read_text(encoding="utf-8"),
    "DU_H": DU_H.read_text(encoding="utf-8"),
}

required = (
    ("SU", "TELEMETRY PRE/POST V1"),
    ("SU", "[SU TELEMETRY] PRE SNAPSHOT"),
    ("SU", "[SU TELEMETRY] POST PENDING"),
    ("SU", "[SU TELEMETRY] POST -> rBS"),
    ("DU", "DU_Latency_PostChannelPending"),
    ("DU", "[DU TELEMETRY] LOCK TRONG PHIEN"),
    ("DU", "[DU TELEMETRY] POST GPS_PHIEN"),
    ("DU", "[DU TELEMETRY] POST CHANNEL SU-DU"),
    ("DU_CPP", "void Xoa_BaoCao_Kenh_SU_DU_DangCho()"),
    ("DU_H", "void Xoa_BaoCao_Kenh_SU_DU_DangCho();"),
)

for key, marker in required:
    if marker not in checks[key]:
        raise RuntimeError(
            f"[STOP] Sanity fail: {key} thieu {marker!r}"
        )

for key, txt in checks.items():
    if "TYPE_BAO_CAO_BAO_MAT" in txt or "Gui_BaoCao_BaoMat_DangCho" in txt:
        raise RuntimeError(
            f"[STOP] {key} bi xuat hien lai security telemetry 0x1A"
        )

print()
print("[DONE] TELEMETRY PRE/POST V1")
print("[PRE] Dung telemetry IDLE gan nhat; KHONG phat them khi bam PTT")
print("[LOCK] RECORD/SESSION/VOICE/FEC/PLAY/HMI: KHONG telemetry")
print("[POST-SU] Ep 1 beacon GPS SU sau khi HMI/radio ranh")
print("[POST-DU] Gui GPS_PHIEN sau session; khong gui GPS periodic lap ngay")
print("[POST-CHANNEL] Bo snapshot PRE cu; cho beacon SU moi roi report RSSI/SNR")
print("[KEEP] AES-128-GCM / ARQ / FEC / packet format / watchdog")
print("[LATENCY] Khong them packet vao critical path cua voice")
print("[NEXT] Build SU + DU. Neu ca hai SUCCESS moi upload ca SU va DU.")
