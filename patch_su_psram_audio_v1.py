from pathlib import Path
from datetime import datetime
import shutil
import re

ROOT = Path(__file__).resolve().parent
SU_MAIN = ROOT / "SU" / "src" / "main.cpp"

if not SU_MAIN.exists():
    raise SystemExit(f"[FAIL] Khong tim thay: {SU_MAIN}")

text = SU_MAIN.read_text(encoding="utf-8")

if "SU AUDIO PSRAM V1" in text:
    raise SystemExit("[FAIL] SU AUDIO PSRAM V1 co ve da duoc patch.")

for marker in (
    "uint8_t *Kho_Chua_AmThanh;",
    "MAX_KHUNG_THOAI",
    "heap_caps_malloc",
    "MALLOC_CAP_8BIT",
    "BAT DAU GHI AM VAO RAM",
):
    if marker not in text:
        raise SystemExit(
            f"[FAIL] SU/main.cpp khong dung baseline mong doi: thieu {marker!r}"
        )

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup = ROOT / f"backup_before_su_psram_audio_v1_{stamp}"
backup.mkdir(parents=True, exist_ok=False)

dst = backup / SU_MAIN.relative_to(ROOT)
dst.parent.mkdir(parents=True, exist_ok=True)
shutil.copy2(SU_MAIN, dst)

print(f"[BACKUP] {backup}")

# 1) Include heap caps explicitly
if '#include "esp_heap_caps.h"' not in text:
    anchor = '#include "esp_idf_version.h"\n'
    if anchor not in text:
        anchor = '#include "esp_system.h"\n'
    if anchor not in text:
        raise RuntimeError("[STOP] Khong tim thay anchor include ESP")

    text = text.replace(
        anchor,
        anchor + '#include "esp_heap_caps.h"\n',
        1
    )
    print("[PATCH] add esp_heap_caps.h")
else:
    print("[SKIP] esp_heap_caps.h da co")

# 2) Add state next to audio buffer declaration
old = '''uint8_t *Kho_Chua_AmThanh;

uint32_t tong_so_khung_da_ghi = 0;
'''

new = '''uint8_t *Kho_Chua_AmThanh;

// ========================================================
// SU AUDIO PSRAM V1
//
// Kho_Chua_AmThanh chua cac frame Speex da nen:
//   MAX_KHUNG_THOAI * 20 byte = 3000 * 20 = 60000 byte.
//
// Uu tien cap phat tu PSRAM neu PSRAM dang kha dung.
// Neu board/config khong expose PSRAM, firmware fallback ve
// internal RAM de khong lam brick thiet bi; log se canh bao ro.
//
// Khong thay packet/codec/AES/FEC/ARQ.
// Khong tang MAX_KHUNG_THOAI o patch nay.
// ========================================================
static bool su_audio_buffer_in_psram = false;
static size_t su_audio_buffer_bytes =
    (size_t)MAX_KHUNG_THOAI * (size_t)SPEEX_BYTES_PER_FRAME;

uint32_t tong_so_khung_da_ghi = 0;
'''

if old not in text:
    raise RuntimeError(
        "[STOP] Khong tim thay block khai bao Kho_Chua_AmThanh"
    )

text = text.replace(old, new, 1)
print("[PATCH] add SU PSRAM audio state")

# 3) Replace old allocation block
start = text.find("    // ====================================================\n    // CẤP PHÁT RAM")
if start == -1:
    start = text.find("    Kho_Chua_AmThanh =")

end_marker = '''    Serial.println(
        "[SU] Khoi tao thanh cong!"
    );
'''
end = text.find(end_marker, start if start != -1 else 0)

if start == -1 or end == -1:
    raise RuntimeError(
        "[STOP] Khong xac dinh an toan block cap phat audio. Khong sua."
    )

end += len(end_marker)

replacement = '''    // ====================================================
    // SU AUDIO PSRAM V1
    // PSRAM-FIRST, SAFE FALLBACK
    // ====================================================

    const size_t psram_total =
        heap_caps_get_total_size(
            MALLOC_CAP_SPIRAM
        );

    const size_t psram_free_before =
        heap_caps_get_free_size(
            MALLOC_CAP_SPIRAM
        );

    const size_t internal_free_before =
        heap_caps_get_free_size(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
        );

    Serial.printf(
        "[SU PSRAM] TOTAL=%u | FREE BEFORE=%u | AUDIO NEED=%u bytes\\n",
        (unsigned int)psram_total,
        (unsigned int)psram_free_before,
        (unsigned int)su_audio_buffer_bytes
    );

    if (
        psram_total > 0
        &&
        psram_free_before >= su_audio_buffer_bytes
    )
    {
        Kho_Chua_AmThanh =
            (uint8_t *)
            heap_caps_malloc(
                su_audio_buffer_bytes,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
            );

        if (Kho_Chua_AmThanh != NULL)
        {
            su_audio_buffer_in_psram = true;
        }
    }

    // Fallback an toan:
    // Neu SU thuc te khong co PSRAM, hoac PlatformIO chua expose PSRAM,
    // van giu he thong chay nhu baseline cu de test/diagnose.
    if (Kho_Chua_AmThanh == NULL)
    {
        Serial.println(
            "[SU PSRAM WARN] KHONG CAP PHAT DUOC PSRAM -> FALLBACK INTERNAL RAM"
        );

        Kho_Chua_AmThanh =
            (uint8_t *)
            heap_caps_malloc(
                su_audio_buffer_bytes,
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            );

        su_audio_buffer_in_psram = false;
    }

    if (Kho_Chua_AmThanh == NULL)
    {
        Serial.println(
            "[SU ERROR] CAP PHAT AUDIO BUFFER THAT BAI!"
        );

        Serial.printf(
            "[SU MEM] PSRAM_FREE=%u | INTERNAL_FREE=%u\\n",
            (unsigned int)
            heap_caps_get_free_size(
                MALLOC_CAP_SPIRAM
            ),
            (unsigned int)
            heap_caps_get_free_size(
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            )
        );

        Serial.println(
            "[SU SELF-HEAL] audio buffer init fail -> reboot sau 1s"
        );

        delay(1000);
        ESP.restart();
    }

    Serial.printf(
        "[SU AUDIO BUFFER] %s | SIZE=%u bytes | MAX_AUDIO=%u ms\\n",
        su_audio_buffer_in_psram ? "PSRAM" : "INTERNAL_RAM",
        (unsigned int)su_audio_buffer_bytes,
        (unsigned int)(MAX_KHUNG_THOAI * 20U)
    );

    Serial.printf(
        "[SU PSRAM] FREE AFTER=%u | INTERNAL FREE AFTER=%u\\n",
        (unsigned int)
        heap_caps_get_free_size(
            MALLOC_CAP_SPIRAM
        ),
        (unsigned int)
        heap_caps_get_free_size(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
        )
    );

    Serial.println(
        "[SU] Khoi tao thanh cong!"
    );
'''

text = text[:start] + replacement + text[end:]
print("[PATCH] PSRAM-first audio buffer allocation")

# 4) Make recording log truthful
old_log = '''        Serial.println(
            ">> BAT DAU GHI AM VAO RAM..."
        );
'''

new_log = '''        Serial.printf(
            ">> BAT DAU GHI AM -> SPEEX BUFFER %s...\\n",
            su_audio_buffer_in_psram ? "PSRAM" : "INTERNAL_RAM"
        );
'''

if old_log in text:
    text = text.replace(old_log, new_log, 1)
    print("[PATCH] recording log reports actual memory region")
else:
    print("[WARN] Khong tim thay log BAT DAU GHI AM cu; bo qua log rename")

# 5) Sanity
required = (
    '#include "esp_heap_caps.h"',
    "SU AUDIO PSRAM V1",
    "MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT",
    "[SU PSRAM] TOTAL=",
    "[SU AUDIO BUFFER] %s",
    "FALLBACK INTERNAL RAM",
)

for marker in required:
    if marker not in text:
        raise RuntimeError(f"[STOP] Sanity fail: thieu {marker!r}")

old_alloc = '''heap_caps_malloc(
            MAX_KHUNG_THOAI * 20,
            MALLOC_CAP_8BIT
        )'''
if old_alloc in text:
    raise RuntimeError("[STOP] Allocation cu MALLOC_CAP_8BIT van con")

SU_MAIN.write_text(text, encoding="utf-8")

print()
print("[DONE] SU AUDIO PSRAM V1")
print("[MODE] PSRAM-first; neu PSRAM khong co thi fallback internal RAM co canh bao")
print("[SIZE] GIU NGUYEN 3000 frame ~ 60 giay, KHONG tang thoi luong o buoc nay")
print("[LATENCY] khong doi codec/packet/AES/FEC/ARQ; chi doi noi luu frame Speex")
print("[NEXT] Build SU -> upload SU -> gui log boot [SU PSRAM] / [SU AUDIO BUFFER].")
