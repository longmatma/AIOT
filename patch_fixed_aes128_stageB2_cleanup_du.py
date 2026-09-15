from __future__ import annotations

from pathlib import Path
from datetime import datetime
import shutil

ROOT = Path(__file__).resolve().parent
DU = ROOT / "DU" / "src"

MAIN = DU / "main.cpp"
NHAN_CPP = DU / "nhan_lora.cpp"
NHAN_H = DU / "nhan_lora.h"

for p in (MAIN, NHAN_CPP, NHAN_H):
    if not p.exists():
        raise SystemExit(f"[FAIL] Khong tim thay: {p}")

# Guard: B1 fixed AES phai da ton tai.
main_text = MAIN.read_text(encoding="utf-8")
if "[DU CRYPTO] FIXED AES-128-GCM | SESSION KEY THEO SESSION_ID" not in main_text:
    raise SystemExit(
        "[FAIL] DU chua o Fixed AES B1. Dung lai de tranh patch nham version."
    )

# Guard: Stage 3B1A telemetry phai con de B2 don.
if "Dat_BaoCao_BaoMat_DangCho" not in main_text:
    raise SystemExit(
        "[FAIL] Khong thay Stage 3B1A security telemetry trong DU/main.cpp. "
        "Co the da duoc xoa roi."
    )

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup = ROOT / f"backup_before_fixed_aes128_stageB2_{stamp}"
backup.mkdir(parents=True, exist_ok=False)

for p in (MAIN, NHAN_CPP, NHAN_H):
    dst = backup / p.relative_to(ROOT)
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dst)

print(f"[BACKUP] {backup}")


def reverse_once(text: str, current: str, baseline: str, label: str) -> str:
    n = text.count(current)
    if n != 1:
        raise RuntimeError(
            f"[STOP] {label}: can 1 block, tim thay {n}. Khong sua."
        )
    print(f"[PATCH] {label}")
    return text.replace(current, baseline, 1)


# ============================================================
# 1) nhan_lora.h - xoa packet/API 0x1A
# ============================================================
t = NHAN_H.read_text(encoding="utf-8")

current = '''bool Gui_BaoCao_Kenh_SU_DU_DangCho();


// =====================================================
// STAGE 3B1A - SECURITY TELEMETRY DU -> rBS
//
// SHADOW ONLY:
// packet nay CHUA co authentication rieng, nen TUYET DOI
// chua duoc dung de closed-loop doi AES.
// =====================================================
#define TYPE_BAO_CAO_BAO_MAT 0x1A
#define SIZE_BAO_CAO_BAO_MAT 28

void Dat_BaoCao_BaoMat_DangCho(
    uint64_t session_id,
    uint16_t voice_gcm_fail,
    uint16_t fec_gcm_fail,
    uint16_t replay_suspect,
    uint16_t voice_gcm_ok,
    uint16_t fec_gcm_ok,
    uint32_t highest_seq
);

bool Gui_BaoCao_BaoMat_DangCho();

#endif
'''

baseline = '''bool Gui_BaoCao_Kenh_SU_DU_DangCho();

#endif
'''

t = reverse_once(
    t,
    current,
    baseline,
    "nhan_lora.h remove security telemetry 0x1A API",
)
NHAN_H.write_text(t, encoding="utf-8")


# ============================================================
# 2) nhan_lora.cpp - xoa pending state
# ============================================================
t = NHAN_CPP.read_text(encoding="utf-8")

current = '''static portMUX_TYPE KenhSUDU_Mux = portMUX_INITIALIZER_UNLOCKED;


// =====================================================
// STAGE 3B1A - SECURITY REPORT PENDING
//
// Decoder chi copy snapshot vao RAM.
// Task telemetry uu tien thap moi TX khi radio ranh.
// =====================================================
struct BaoCaoBaoMatPending
{
    bool pending;
    uint64_t session_id;
    uint16_t voice_gcm_fail;
    uint16_t fec_gcm_fail;
    uint16_t replay_suspect;
    uint16_t voice_gcm_ok;
    uint16_t fec_gcm_ok;
    uint32_t highest_seq;
};

static BaoCaoBaoMatPending bao_cao_bao_mat_pending = {};
static portMUX_TYPE BaoMatReportMux = portMUX_INITIALIZER_UNLOCKED;
'''

baseline = '''static portMUX_TYPE KenhSUDU_Mux = portMUX_INITIALIZER_UNLOCKED;
'''

t = reverse_once(
    t,
    current,
    baseline,
    "nhan_lora.cpp remove security pending state",
)


# ============================================================
# 3) nhan_lora.cpp - xoa setter + sender 0x1A
# ============================================================
start_marker = '''// =====================================================
// STAGE 3B1A - DAT SECURITY SNAPSHOT DANG CHO
// =====================================================
'''

end_marker = '''// =====================================================
// GUI SNAPSHOT KENH SU -> DU VE rBS
'''

start = t.find(start_marker)
end = t.find(end_marker)

if start < 0 or end < 0 or end <= start:
    raise RuntimeError(
        "[STOP] Khong tim thay dung block Stage 3B1A trong nhan_lora.cpp"
    )

t = t[:start] + t[end:]
print("[PATCH] nhan_lora.cpp remove security setter/sender 0x1A")

NHAN_CPP.write_text(t, encoding="utf-8")


# ============================================================
# 4) main.cpp - xoa counters/replay heuristic
# ============================================================
t = MAIN.read_text(encoding="utf-8")

current = '''bool da_co_session = false;


// =====================================================
// STAGE 3B1A - SECURITY COUNTERS THEO SESSION
//
// Khong dung RSSI/SNR/H.
// Replay heuristic:
// - chi xet packet DA GCM OK;
// - packet cu cach highest <= 16: bo qua de tranh nham ARQ;
// - packet cu hon 16 packet: REPLAY_SUSPECT.
// =====================================================
static uint16_t sec_voice_gcm_fail = 0;
static uint16_t sec_fec_gcm_fail = 0;
static uint16_t sec_replay_suspect = 0;
static uint16_t sec_voice_gcm_ok = 0;
static uint16_t sec_fec_gcm_ok = 0;
static uint32_t sec_highest_auth_seq = 0;
static bool sec_have_highest_auth_seq = false;

static void Sec_Inc_U16(uint16_t &v)
{
    if (v < 0xFFFFU)
        ++v;
}

static void Sec_Reset_Session()
{
    sec_voice_gcm_fail = 0;
    sec_fec_gcm_fail = 0;
    sec_replay_suspect = 0;
    sec_voice_gcm_ok = 0;
    sec_fec_gcm_ok = 0;
    sec_highest_auth_seq = 0;
    sec_have_highest_auth_seq = false;
}
'''

baseline = '''bool da_co_session = false;
'''

t = reverse_once(
    t,
    current,
    baseline,
    "main.cpp remove Stage 3B1A counters",
)

t = reverse_once(
    t,
    '''            Reset_FEC_Group();
            Reset_ThongKe_OLED_DU();
            Sec_Reset_Session();

            HienThi_DU_DangNhan(
''',
    '''            Reset_FEC_Group();
            Reset_ThongKe_OLED_DU();

            HienThi_DU_DangNhan(
''',
    "main.cpp remove security counter reset",
)

t = reverse_once(
    t,
    '''                Serial.printf(
                    "[DU FEC GCM FAIL] GROUP=%u\\n",
                    group_start
                );

                Sec_Inc_U16(sec_fec_gcm_fail);

                // Không dùng parity không xác thực.
                continue;
''',
    '''                Serial.printf(
                    "[DU FEC GCM FAIL] GROUP=%u\\n",
                    group_start
                );

                // Không dùng parity không xác thực.
                continue;
''',
    "main.cpp remove FEC FAIL telemetry counter",
)

t = reverse_once(
    t,
    '''                // Không dùng parity không xác thực.
                continue;
            }

            Sec_Inc_U16(sec_fec_gcm_ok);

            if (
                !fec_group.active
            )
''',
    '''                // Không dùng parity không xác thực.
                continue;
            }

            if (
                !fec_group.active
            )
''',
    "main.cpp remove FEC OK telemetry counter",
)

t = reverse_once(
    t,
    '''                    Sec_Inc_U16(sec_voice_gcm_ok);

                    Serial.printf(
                        "[DU FEC RECOVER + VOICE GCM OK] SEQ=%u | SLOT=%d\\n",
                        recovered_seq,
                        missing_slot
                    );
''',
    '''                    Serial.printf(
                        "[DU FEC RECOVER + VOICE GCM OK] SEQ=%u | SLOT=%d\\n",
                        recovered_seq,
                        missing_slot
                    );
''',
    "main.cpp remove recovered VOICE OK counter",
)

t = reverse_once(
    t,
    '''                    Sec_Inc_U16(sec_voice_gcm_fail);

                    Serial.printf(
                        "[DU FEC RECOVER BUT VOICE GCM FAIL] SEQ=%u | SLOT=%d\\n",
                        recovered_seq,
                        missing_slot
                    );
''',
    '''                    Serial.printf(
                        "[DU FEC RECOVER BUT VOICE GCM FAIL] SEQ=%u | SLOT=%d\\n",
                        recovered_seq,
                        missing_slot
                    );
''',
    "main.cpp remove recovered VOICE FAIL counter",
)

t = reverse_once(
    t,
    '''            Serial.printf(
                "[DU VOICE GCM FAIL -> XEM NHU MISSING] SEQ=%u\\n",
                seq
            );

            Sec_Inc_U16(sec_voice_gcm_fail);

            // Không dùng LAST/frame/group metadata từ packet GCM fail.
''',
    '''            Serial.printf(
                "[DU VOICE GCM FAIL -> XEM NHU MISSING] SEQ=%u\\n",
                seq
            );

            // Không dùng LAST/frame/group metadata từ packet GCM fail.
''',
    "main.cpp remove VOICE FAIL telemetry counter",
)

current = '''        Sec_Inc_U16(sec_voice_gcm_ok);

        if (
            sec_have_highest_auth_seq
            &&
            seq < sec_highest_auth_seq
            &&
            (sec_highest_auth_seq - seq) > 16U
        )
        {
            Sec_Inc_U16(sec_replay_suspect);

            Serial.printf(
                "[DU SECURITY] REPLAY_SUSPECT | SEQ=%u | HIGHEST=%u\\n",
                seq,
                sec_highest_auth_seq
            );
        }

        if (
            !sec_have_highest_auth_seq
            ||
            seq > sec_highest_auth_seq
        )
        {
            sec_highest_auth_seq = seq;
            sec_have_highest_auth_seq = true;
        }

        uint32_t group_start =
            seq
            -
            (
                seq
                % FEC_DATA_PER_GROUP
            );
'''

baseline = '''        uint32_t group_start =
            seq
            -
            (
                seq
                % FEC_DATA_PER_GROUP
            );
'''

t = reverse_once(
    t,
    current,
    baseline,
    "main.cpp remove VOICE OK/replay telemetry heuristic",
)

t = reverse_once(
    t,
    '''        // Stage 3B1A: chi COPY snapshot vao pending RAM.
        // Khong TX LoRa trong audio task.
        Dat_BaoCao_BaoMat_DangCho(
            du_last_played_session_id,
            sec_voice_gcm_fail,
            sec_fec_gcm_fail,
            sec_replay_suspect,
            sec_voice_gcm_ok,
            sec_fec_gcm_ok,
            sec_have_highest_auth_seq ? sec_highest_auth_seq : 0U
        );

        // HMI module tu tao AUTO_ACK va mo quyen NACK
''',
    '''        // HMI module tu tao AUTO_ACK va mo quyen NACK
''',
    "main.cpp remove queued security snapshot",
)

t = reverse_once(
    t,
    '''        if (!radio_dang_ban)
        {
            // Security telemetry uu tien hon measurement-only.
            // Neu vua gui security thi de report kenh sang vong sau.
            if (!Gui_BaoCao_BaoMat_DangCho())
            {
                Gui_BaoCao_Kenh_SU_DU_DangCho();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(25));
''',
    '''        if (!radio_dang_ban)
            Gui_BaoCao_Kenh_SU_DU_DangCho();

        vTaskDelay(pdMS_TO_TICKS(25));
''',
    "main.cpp restore channel-report-only telemetry task",
)

MAIN.write_text(t, encoding="utf-8")


# ============================================================
# 5) SANITY
# ============================================================
for p in (MAIN, NHAN_CPP, NHAN_H):
    txt = p.read_text(encoding="utf-8")

    forbidden = (
        "TYPE_BAO_CAO_BAO_MAT",
        "SIZE_BAO_CAO_BAO_MAT",
        "Dat_BaoCao_BaoMat_DangCho",
        "Gui_BaoCao_BaoMat_DangCho",
        "BaoCaoBaoMatPending",
        "sec_voice_gcm_fail",
        "sec_fec_gcm_fail",
        "sec_replay_suspect",
        "sec_voice_gcm_ok",
        "sec_fec_gcm_ok",
        "sec_highest_auth_seq",
        "REPLAY_SUSPECT",
    )

    for marker in forbidden:
        if marker in txt:
            raise RuntimeError(
                f"[STOP] Con symbol Stage 3B1A {marker!r} trong {p}"
            )

# Bao dam diagnostic GCM that su van con.
final_main = MAIN.read_text(encoding="utf-8")

for required in (
    "[DU VOICE GCM FAIL -> XEM NHU MISSING]",
    "[DU FEC GCM FAIL]",
    "[DU VOICE GCM OK]",
):
    if required not in final_main:
        raise RuntimeError(
            f"[STOP] Mat diagnostic GCM can giu: {required}"
        )

print()
print("[DONE] FIXED AES-128-GCM STAGE B2 CLEANUP")
print("[REMOVE] DU security telemetry packet 0x1A")
print("[REMOVE] adaptive/replay counters")
print("[KEEP] AES-128-GCM authentication + GCM FAIL/OK diagnostics")
print("[KEEP] FEC / ARQ / GPS / channel telemetry 0x19")
print("[NEXT] Build DU. Neu SUCCESS thi upload DU.")
