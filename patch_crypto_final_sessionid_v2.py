from pathlib import Path
from datetime import datetime
import shutil
import re

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

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup = ROOT / f"backup_before_crypto_final_sessionid_v2_{stamp}"
backup.mkdir(parents=True, exist_ok=False)

for p in (SU_MAIN, DONG_CPP, DONG_H):
    dst = backup / p.relative_to(ROOT)
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dst)

print(f"[BACKUP] {backup}")


def rep(text, old, new, label):
    n = text.count(old)
    if n != 1:
        raise RuntimeError(f"[STOP] {label}: can dung 1 vi tri, tim thay {n}")
    print(f"[PATCH] {label}")
    return text.replace(old, new, 1)


# A) Preferences
if "#include <Preferences.h>" not in cpp:
    old = '#include "esp_system.h"\n'
    if old not in cpp:
        raise RuntimeError("[STOP] Khong tim thay #include esp_system.h")
    cpp = cpp.replace(old, old + "#include <Preferences.h>\n", 1)
    print("[PATCH] dong_goi.cpp add Preferences")
else:
    print("[SKIP] Preferences da co")


# B) Persistent SESSION_ID allocator
if "bool KhoiTao_Session_ID_BenVung()" not in cpp:
    marker_start = "// =====================================================\n// SESSION MỚI"
    marker_end = "// =====================================================\n// SESSION_START = 12 BYTE"

    if marker_start not in cpp or marker_end not in cpp:
        raise RuntimeError(
            "[STOP] Khong tim thay anchor SESSION MOI / SESSION_START trong dong_goi.cpp"
        )

    start = cpp.index(marker_start)
    end = cpp.index(marker_end, start)

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
//   - ghi high-water mark vao NVS TRUOC;
//   - neu mat dien, ID chua dung bi bo qua, KHONG bi reuse;
//   - chi ghi NVS 1 lan/boot + moi 4096 session.
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
        persisted_end + SESSION_COUNTER_BLOCK;

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
        (((uint64_t)counter32) << 32)
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
    print("[PATCH] dong_goi.cpp persistent non-reuse SESSION_ID")
else:
    print("[SKIP] Persistent SESSION_ID allocator da co")


# C) Header
if "bool KhoiTao_Session_ID_BenVung();" not in hdr:
    target = "uint64_t Tao_Session_Moi();"
    if target not in hdr:
        raise RuntimeError("[STOP] Khong tim thay Tao_Session_Moi() trong dong_goi.h")
    hdr = hdr.replace(
        target,
        "// Khoi tao SESSION_ID ben vung tu NVS tai boot.\n"
        "bool KhoiTao_Session_ID_BenVung();\n\n"
        + target,
        1
    )
    print("[PATCH] dong_goi.h declare persistent SID init")
else:
    print("[SKIP] Header SID init da co")


# D) Init in setup
if "if (!KhoiTao_Session_ID_BenVung())" not in main:
    anchor = '''    delay(100);
    SU_WDT_Init();
    SU_WDT_Feed();
'''
    if anchor not in main:
        raise RuntimeError("[STOP] Khong tim thay anchor setup WDT trong SU/main.cpp")

    insert = anchor + '''
    // ====================================================
    // CRYPTO FINAL SESSION-ID V1
    // Reserve persistent SESSION_ID block tai BOOT.
    // NVS write khong nam tren critical path PTT.
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
'''
    main = main.replace(anchor, insert, 1)
    print("[PATCH] SU init persistent session allocator at boot")
else:
    print("[SKIP] SU persistent SID init da co")


# E) Remove GPS block between READY and VOICE
if "Gui_GPS_REPORT_SU(session_id_tx, du_lieu_gps_su);" in main:
    pattern = re.compile(
        r'''
        \n[ \t]*DuLieuGPS_SU[ \t]+du_lieu_gps_su[ \t]*=[ \t]*Lay_DuLieu_GPS_SU\(\);[ \t]*\n
        [ \t]*In_TrangThai_GPS_SU\(du_lieu_gps_su\);[ \t]*\n
        (?:[ \t]*\n)?
        [ \t]*Dat_LED_SU\(true\);[ \t]*\n
        [ \t]*Gui_GPS_REPORT_SU\(session_id_tx,[ \t]*du_lieu_gps_su\);[ \t]*\n
        [ \t]*Dat_LED_SU\(false\);[ \t]*\n
        (?:[ \t]*\n)?
        (?:[ \t]*//[^\n]*\n)?
        [ \t]*delay\(8\);[ \t]*\n
        ''',
        re.VERBOSE
    )

    main2, n = pattern.subn("\n", main, count=1)

    if n != 1:
        lines = main.splitlines(True)
        idx = None
        for i, line in enumerate(lines):
            if "Gui_GPS_REPORT_SU(session_id_tx, du_lieu_gps_su);" in line:
                idx = i
                break

        if idx is None:
            raise RuntimeError("[STOP] Khong tim thay dong Gui_GPS_REPORT_SU")

        start = idx
        while start >= 0 and "DuLieuGPS_SU du_lieu_gps_su" not in lines[start]:
            start -= 1

        end = idx
        while end < len(lines) and "delay(8);" not in lines[end]:
            end += 1

        if (
            start < 0
            or end >= len(lines)
            or "DuLieuGPS_SU du_lieu_gps_su" not in lines[start]
            or "delay(8);" not in lines[end]
        ):
            raise RuntimeError(
                "[STOP] Khong xac dinh an toan block GPS READY->VOICE"
            )

        del lines[start:end + 1]
        main2 = "".join(lines)

    main = main2

    if "[SU TELEMETRY] READY->VOICE | KHONG GPS CHEN GIUA" not in main:
        ready_anchor = '''            Serial.printf(
                "[SU SESSION READY] DU DA SAN SANG | SESSION=%016llX\\n",
                (unsigned long long)session_id_tx
            );
'''
        if ready_anchor in main:
            main = main.replace(
                ready_anchor,
                ready_anchor
                + '''
            Serial.println(
                "[SU TELEMETRY] READY->VOICE | KHONG GPS CHEN GIUA"
            );
''',
                1
            )
        else:
            print("[WARN] Da bo GPS nhung khong chen duoc log READY->VOICE")

    print("[PATCH] remove GPS_REPORT_SU + delay(8) between READY and VOICE")
else:
    print("[SKIP] GPS_REPORT_SU READY->VOICE da khong con")


# WRITE only after all patch steps succeeded
DONG_CPP.write_text(cpp, encoding="utf-8")
DONG_H.write_text(hdr, encoding="utf-8")
SU_MAIN.write_text(main, encoding="utf-8")

# SANITY
final_cpp = DONG_CPP.read_text(encoding="utf-8")
final_hdr = DONG_H.read_text(encoding="utf-8")
final_main = SU_MAIN.read_text(encoding="utf-8")

required = (
    (final_cpp, "#include <Preferences.h>"),
    (final_cpp, "SESSION_COUNTER_BLOCK = 4096UL"),
    (final_cpp, "bool KhoiTao_Session_ID_BenVung()"),
    (final_cpp, '"sid_hi"'),
    (final_cpp, "SID_COUNTER=%u"),
    (final_hdr, "bool KhoiTao_Session_ID_BenVung();"),
    (final_main, "if (!KhoiTao_Session_ID_BenVung())"),
)

for text, marker in required:
    if marker not in text:
        raise RuntimeError(f"[STOP] Sanity fail: thieu {marker!r}")

if "Gui_GPS_REPORT_SU(session_id_tx, du_lieu_gps_su);" in final_main:
    raise RuntimeError("[STOP] GPS_REPORT_SU READY->VOICE van con")

print()
print("[DONE] CRYPTO FINAL SESSION-ID V2")
print("[AES] Fixed AES-128-GCM GIU NGUYEN")
print("[SID] HIGH32=NVS counter | LOW32=esp_random")
print("[NVS] reserve 4096 IDs tai boot; reboot khong reuse khi NVS con nguyen")
print("[TELEMETRY] GPS_REPORT_SU + delay(8) giua READY->VOICE da bo")
print("[LATENCY] khong them NVS write vao PTT critical path")
print("[DU/rBS/STM32] khong can sua")
print("[NEXT] Build SU. Neu SUCCESS -> upload CHI SU.")
