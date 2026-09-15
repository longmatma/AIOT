from pathlib import Path
from datetime import datetime
import shutil

ROOT = Path(__file__).resolve().parent

SU_MAIN = ROOT / "SU" / "src" / "main.cpp"
DONG_CPP = ROOT / "SU" / "src" / "dong_goi.cpp"
DONG_H = ROOT / "SU" / "src" / "dong_goi.h"

for p in (SU_MAIN, DONG_CPP, DONG_H):
    if not p.exists():
        raise SystemExit(f"[FAIL] Khong tim thay: {p}")

main = SU_MAIN.read_text(encoding="utf-8")
cpp = DONG_CPP.read_text(encoding="utf-8")
hdr = DONG_H.read_text(encoding="utf-8")

if "CRYPTO FINAL SESSION-ID V1" in main or "CRYPTO FINAL SESSION-ID V1" in cpp:
    raise SystemExit("[FAIL] Patch CRYPTO FINAL SESSION-ID V1 co ve da chay.")

for marker in (
    "uint64_t Tao_Session_Moi()",
    "esp_random()",
    "session_id_hien_tai",
):
    if marker not in cpp:
        raise SystemExit(f"[FAIL] dong_goi.cpp sai baseline: thieu {marker!r}")

for marker in (
    "uint64_t Tao_Session_Moi();",
    "void Tao_GoiTin_SessionStart",
):
    if marker not in hdr:
        raise SystemExit(f"[FAIL] dong_goi.h sai baseline: thieu {marker!r}")

for marker in (
    "[SU SESSION READY] DU DA SAN SANG",
    "Gui_GPS_REPORT_SU(session_id_tx, du_lieu_gps_su);",
    "delay(8);",
    "KhoiTao_HMI_SU();",
):
    if marker not in main:
        raise SystemExit(f"[FAIL] SU/main.cpp sai baseline: thieu {marker!r}")

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup = ROOT / f"backup_before_crypto_final_sessionid_v1_{stamp}"
backup.mkdir(parents=True, exist_ok=False)

for p in (SU_MAIN, DONG_CPP, DONG_H):
    dst = backup / p.relative_to(ROOT)
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dst)

print(f"[BACKUP] {backup}")


def rep(text, old, new, label):
    n = text.count(old)
    if n != 1:
        raise RuntimeError(
            f"[STOP] {label}: can dung 1 vi tri, tim thay {n}"
        )
    print(f"[PATCH] {label}")
    return text.replace(old, new, 1)


old = '#include "esp_system.h"\n#include <string.h>\n'
new = '#include "esp_system.h"\n#include <Preferences.h>\n#include <string.h>\n'
cpp = rep(cpp, old, new, "dong_goi.cpp add Preferences")

start = cpp.index("// =====================================================\n// SESSION MỚI")
end = cpp.index("// =====================================================\n// SESSION_START = 12 BYTE", start)

new_block = r'''// =====================================================
// CRYPTO FINAL SESSION-ID V1
//
// AES-GCM yeu cau: KHONG DUOC reuse cung KEY + IV.
//
// Session key phu thuoc SESSION_ID, con IV = SESSION_ID || nonce32.
// Vi vay SESSION_ID phai tranh lap qua reboot.
//
// Co che:
//   HIGH32 = persistent monotonic counter trong NVS.
//   LOW32  = esp_random().
//
// De giam wear flash:
//   - moi lan reserve 4096 counter truoc khi dung;
//   - ghi "high-water mark" vao NVS TRUOC;
//   - neu mat dien, cac ID chua dung bi bo qua, KHONG bi reuse.
//   - chi ghi NVS 1 lan/boot + moi 4096 session.
//
// Luu y:
//   Neu NVS bi xoa co chu dich/factory-reset, counter co the ve 0.
//   LOW32 random van giam manh xac suat trung SESSION_ID cu.
// =====================================================

static constexpr uint32_t SESSION_COUNTER_BLOCK = 4096UL;

static uint32_t session_counter_next = 0;
static uint32_t session_counter_end = 0;
static bool session_allocator_ready = false;


static bool Reserve_Session_Counter_Block()
{
    Preferences prefs;

    if (!prefs.begin("vedc_crypto", false))
    {
        Serial.println(
            "[SU CRYPTO FATAL] KHONG MO DUOC NVS namespace vedc_crypto"
        );
        return false;
    }

    uint32_t persisted_end =
        prefs.getUInt("sid_hi", 0UL);

    if (
        persisted_end
        >
        (0xFFFFFFFFUL - SESSION_COUNTER_BLOCK)
    )
    {
        prefs.end();

        Serial.println(
            "[SU CRYPTO FATAL] SESSION COUNTER DA GAN TRAN 32-BIT"
        );
        return false;
    }

    const uint32_t new_end =
        persisted_end
        +
        SESSION_COUNTER_BLOCK;

    size_t written =
        prefs.putUInt(
            "sid_hi",
            new_end
        );

    prefs.end();

    if (written != sizeof(uint32_t))
    {
        Serial.println(
            "[SU CRYPTO FATAL] GHI NVS sid_hi THAT BAI"
        );
        return false;
    }

    session_counter_next =
        persisted_end + 1UL;

    session_counter_end =
        new_end;

    session_allocator_ready =
        true;

    Serial.printf(
        "[SU CRYPTO] SESSION-ID BLOCK RESERVED | COUNTER=%u..%u | SIZE=%u\n",
        (unsigned int)session_counter_next,
        (unsigned int)session_counter_end,
        (unsigned int)SESSION_COUNTER_BLOCK
    );

    return true;
}


bool KhoiTao_Session_ID_BenVung()
{
    if (session_allocator_ready)
        return true;

    return Reserve_Session_Counter_Block();
}


// =====================================================
// SESSION MỚI
// =====================================================

uint64_t Tao_Session_Moi()
{
    if (
        !session_allocator_ready
        ||
        session_counter_next == 0
        ||
        session_counter_next > session_counter_end
    )
    {
        if (!Reserve_Session_Counter_Block())
        {
            Serial.println(
                "[SU CRYPTO FATAL] KHONG CAP DUOC SESSION-ID -> REBOOT"
            );
            Serial.flush();
            delay(200);
            ESP.restart();
            return 0;
        }
    }

    const uint32_t counter32 =
        session_counter_next++;

    const uint32_t random32 =
        esp_random();

    session_id_hien_tai =
        (
            ((uint64_t)counter32) << 32
        )
        |
        (uint64_t)random32;

    so_thu_tu_goi = 0;

    Serial.printf(
        "[SU] NEW SESSION = %016llX | SID_COUNTER=%u\n",
        (unsigned long long)session_id_hien_tai,
        (unsigned int)counter32
    );

    return session_id_hien_tai;
}


'''

cpp = cpp[:start] + new_block + cpp[end:]
print("[PATCH] dong_goi.cpp persistent non-reuse SESSION_ID allocator")

old = '''uint64_t Tao_Session_Moi();

void Tao_GoiTin_SessionStart(
'''
new = '''// Khoi tao bo cap SESSION_ID ben vung tu NVS.
// Goi 1 lan luc boot de NVS write KHONG nam tren critical path PTT.
bool KhoiTao_Session_ID_BenVung();

uint64_t Tao_Session_Moi();

void Tao_GoiTin_SessionStart(
'''
hdr = rep(hdr, old, new, "dong_goi.h declare persistent SID init")

old = '''    delay(100);
    SU_WDT_Init();
    SU_WDT_Feed();

    // GPS NEO-6M doc lien tuc trong task rieng, khong block audio/PTT.
'''
new = '''    delay(100);
    SU_WDT_Init();
    SU_WDT_Feed();

    // ====================================================
    // CRYPTO FINAL SESSION-ID V1
    //
    // Reserve persistent SESSION_ID block tai BOOT de tranh
    // NVS write chen vao PTT_RELEASE -> SESSION_START.
    // Neu NVS loi, fail-closed va reboot.
    // ====================================================
    if (!KhoiTao_Session_ID_BenVung())
    {
        Serial.println(
            "[SU CRYPTO FATAL] KHOI TAO SESSION-ID THAT BAI -> REBOOT"
        );
        Serial.flush();
        delay(1000);
        ESP.restart();
        return;
    }

    SU_WDT_Feed();

    // GPS NEO-6M doc lien tuc trong task rieng, khong block audio/PTT.
'''
main = rep(main, old, new, "SU init persistent session allocator at boot")

old = '''        else
        {
            Serial.printf(
                "[SU SESSION READY] DU DA SAN SANG | SESSION=%016llX\n",
                (unsigned long long)session_id_tx
            );

            // GPS la packet rieng, khong chen vao SESSION_START/VOICE.
            // Gui snapshot moi nhat NGAY SAU READY, truoc VOICE.
            DuLieuGPS_SU du_lieu_gps_su = Lay_DuLieu_GPS_SU();
            In_TrangThai_GPS_SU(du_lieu_gps_su);

            Dat_LED_SU(true);
            Gui_GPS_REPORT_SU(session_id_tx, du_lieu_gps_su);
            Dat_LED_SU(false);

            // Cho rBS doc GPS_REPORT + re-arm RX truoc VOICE dau tien.
            delay(8);
        }
'''
new = '''        else
        {
            Serial.printf(
                "[SU SESSION READY] DU DA SAN SANG | SESSION=%016llX\n",
                (unsigned long long)session_id_tx
            );

            // TELEMETRY PRE/POST FINAL:
            // KHONG gui GPS_REPORT chen giua READY va VOICE.
            // PRE = snapshot idle gan nhat da co.
            // POST = beacon pending sau khi session/HMI ket thuc.
            // Voice packet dau tien duoc gui NGAY.
            Serial.println(
                "[SU TELEMETRY] READY->VOICE | KHONG GPS CHEN GIUA"
            );
        }
'''
main = rep(main, old, new, "remove GPS_REPORT_SU + 8ms gap between READY and VOICE")

DONG_CPP.write_text(cpp, encoding="utf-8")
DONG_H.write_text(hdr, encoding="utf-8")
SU_MAIN.write_text(main, encoding="utf-8")

final_cpp = DONG_CPP.read_text(encoding="utf-8")
final_hdr = DONG_H.read_text(encoding="utf-8")
final_main = SU_MAIN.read_text(encoding="utf-8")

required = (
    (final_cpp, "#include <Preferences.h>"),
    (final_cpp, "SESSION_COUNTER_BLOCK = 4096UL"),
    (final_cpp, "prefs.putUInt("),
    (final_cpp, '"sid_hi"'),
    (final_cpp, "bool KhoiTao_Session_ID_BenVung()"),
    (final_cpp, "SID_COUNTER=%u"),
    (final_hdr, "bool KhoiTao_Session_ID_BenVung();"),
    (final_main, "if (!KhoiTao_Session_ID_BenVung())"),
    (final_main, "[SU TELEMETRY] READY->VOICE | KHONG GPS CHEN GIUA"),
)

for text, marker in required:
    if marker not in text:
        raise RuntimeError(f"[STOP] Sanity fail: thieu {marker!r}")

if "Gui_GPS_REPORT_SU(session_id_tx, du_lieu_gps_su);" in final_main:
    raise RuntimeError(
        "[STOP] GPS_REPORT_SU chen READY->VOICE van con"
    )

print()
print("[DONE] CRYPTO FINAL SESSION-ID V1")
print("[AES] GIU NGUYEN Fixed AES-128-GCM")
print("[SID] HIGH32=NVS monotonic counter | LOW32=esp_random")
print("[NVS] Reserve 4096 ID truoc khi dung -> reboot khong reuse khi NVS con nguyen")
print("[LATENCY] NVS reserve o BOOT, khong chen vao PTT binh thuong")
print("[TELEMETRY] BO GPS_REPORT_SU giua SESSION_READY va VOICE")
print("[PACKET] KHONG doi SESSION/VOICE/FEC size, IV size, tag size")
print("[DU] KHONG can sua code")
print("[rBS/STM32] KHONG can sua")
print("[NEXT] Build SU. Neu SUCCESS -> upload CHI SU -> test 2 session + reboot SU -> test lai.")
