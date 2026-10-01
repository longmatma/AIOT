#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# V2C56G2B - SU SINGLE BACKLOG LQ RESCUE
#
# Mục tiêu:
# - Không drop audio.
# - Không đổi TDMA/rBS/DU.
# - Không đụng phat_lora.cpp, nhan_lora.cpp, JAM lease, P_JAM, pulse, marker.
# - Khi đang SINGLE của chính Pair mà queue tăng cao, capture tạm chuyển HQ -> LQ.
# - Khi queue giảm thấp, tự quay lại HQ.
#
# Ngưỡng:
#   ON  khi queue >= 24 frames (~480 ms)
#   OFF khi queue <= 8 frames (~160 ms)

import argparse
import hashlib
import shutil
from datetime import datetime
from pathlib import Path

TAG = "V2C56G2B_SINGLE_BACKLOG_LQ_RESCUE"

OLD_STATE = '''    bool gui_that_bai = false;

    while (!gui_that_bai)
'''

NEW_STATE = '''    bool gui_that_bai = false;

    // V2C56G2B_SINGLE_BACKLOG_LQ_RESCUE
    // Chi la audio-profile rescue o SINGLE. KHONG lien quan friendly-jamming.
    bool v2c56g2b_lq_rescue = false;

    while (!gui_that_bai)
'''

OLD_PROFILE = '''        // V2C4.6: profile cap nhat ca PREPARE. PREPARE->DUAL doi LQ SOM
        // mot frame; DUAL->SINGLE chi doi HQ sau COMMIT.
        if (su_v2b_beacon.valid)
            su_stream_encode_hq = SU_V2C46_CaptureShouldUseHQ();
'''

NEW_PROFILE = '''        // V2C4.6 + V2C56G2B:
        // - Scheduler van quyet dinh HQ/LQ theo SINGLE/PREPARE/DUAL nhu cu.
        // - Rieng SINGLE cua chinh Pair: neu queue cao, tam capture LQ de tao
        //   headroom. SINGLE gui 2 packet/300ms; 2 LQ packet co the mang 600ms
        //   audio, nen backlog co co hoi giam ma KHONG drop frame cu.
        // - Khi queue xuong thap, tu dong quay lai HQ.
        // - KHONG sua packet, TDMA, AES/FEC hay friendly-jamming.
        if (su_v2b_beacon.valid)
        {
            const bool scheduler_hq = SU_V2C46_CaptureShouldUseHQ();
            const bool single_for_me_rescue =
                SU_V2C2_ModeIsSingleForMe(su_v2b_beacon.schedule_mode);
            const UBaseType_t q_now =
                uxQueueMessagesWaiting(su_stream_frame_queue);

            if (scheduler_hq && single_for_me_rescue)
            {
                if (!v2c56g2b_lq_rescue && q_now >= 24U)
                {
                    v2c56g2b_lq_rescue = true;
                    Serial.printf(
                        "[SU V2C56G2B LQ RESCUE ON] QUEUE=%u | HIGH=24 | LOW=8 | MODE=%u\\n",
                        (unsigned int)q_now,
                        (unsigned int)su_v2b_beacon.schedule_mode
                    );
                }
                else if (v2c56g2b_lq_rescue && q_now <= 8U)
                {
                    v2c56g2b_lq_rescue = false;
                    Serial.printf(
                        "[SU V2C56G2B LQ RESCUE OFF] QUEUE=%u | REASON=RECOVERED\\n",
                        (unsigned int)q_now
                    );
                }

                su_stream_encode_hq = !v2c56g2b_lq_rescue;
            }
            else
            {
                if (v2c56g2b_lq_rescue)
                {
                    Serial.printf(
                        "[SU V2C56G2B LQ RESCUE OFF] QUEUE=%u | REASON=SCHEDULER_LQ_OR_TRANSITION | MODE=%u | NEXT=%u\\n",
                        (unsigned int)q_now,
                        (unsigned int)su_v2b_beacon.schedule_mode,
                        (unsigned int)su_v2b_beacon.transition_target
                    );
                }

                v2c56g2b_lq_rescue = false;
                su_stream_encode_hq = scheduler_hq;
            }
        }
'''


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--project-root", default=".")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    root = Path(args.project_root).expanduser().resolve()
    main_cpp = root / "src" / "main.cpp"
    phat = root / "src" / "phat_lora.cpp"

    if not main_cpp.is_file():
        print(f"[FAIL] Không thấy {main_cpp}")
        return 2
    if not phat.is_file():
        print(f"[FAIL] Không thấy {phat}")
        return 3

    s = main_cpp.read_text(encoding="utf-8", errors="strict")
    phat_hash_before = sha256(phat)

    required = [
        "#define SU_STREAM_FRAME_QUEUE_DEPTH 128",
        "SU_V2C46_CaptureShouldUseHQ",
        "SU_V2C2_ModeIsSingleForMe",
        "V2C4_DUAL_P1_OFFSET_MS",
        "V2C4_DUAL_P2_OFFSET_MS",
        "SU_Stream_TaoVoicePacketTuQueue",
        "SU_Stream_GuiVoiceBlock",
    ]
    missing = [x for x in required if x not in s]
    if missing:
        print("[FAIL] Baseline hiện tại không đúng G2A/V2C4.6 mong đợi. Thiếu:")
        for x in missing:
            print("  -", x)
        return 4

    if TAG in s:
        print(f"[OK] Patch đã có: {TAG}")
        print(f"[JAM_GUARD] phat_lora.cpp SHA256={phat_hash_before}")
        return 0

    c1 = s.count(OLD_STATE)
    c2 = s.count(OLD_PROFILE)

    if c1 != 1 or c2 != 1:
        print(f"[FAIL] Anchor không duy nhất: STATE={c1}, PROFILE={c2}")
        print("[FAIL] KHÔNG sửa file.")
        return 5

    print("[CHECK] Baseline SU G2A queue=128: OK")
    print("[CHECK] V2C4.6 adaptive HQ/LQ: OK")
    print("[PLAN] SINGLE queue >=24 -> capture LQ rescue")
    print("[PLAN] SINGLE queue <=8  -> quay lại HQ")
    print("[PLAN] PREPARE/DUAL giữ logic LQ hiện tại")
    print("[PLAN] Không drop audio, không đổi packet/TDMA/AES/FEC")
    print("[JAM_GUARD] KHÔNG sửa src/phat_lora.cpp")
    print(f"[JAM_GUARD] SHA256 trước={phat_hash_before}")

    if args.check:
        print("[CHECK ONLY] Không có file nào bị sửa.")
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = main_cpp.with_name(f"main.cpp.bak_before_{TAG}_{stamp}")
    shutil.copy2(main_cpp, backup)
    print(f"[BACKUP] {backup}")

    patched = s.replace(OLD_STATE, NEW_STATE, 1)
    patched = patched.replace(OLD_PROFILE, NEW_PROFILE, 1)
    main_cpp.write_text(patched, encoding="utf-8")

    verify = main_cpp.read_text(encoding="utf-8", errors="strict")
    if TAG not in verify or verify.count("LQ RESCUE ON") != 1:
        shutil.copy2(backup, main_cpp)
        print("[FAIL] Verify patch lỗi.")
        print("[ROLLBACK] Đã khôi phục main.cpp.")
        return 6

    phat_hash_after = sha256(phat)
    if phat_hash_after != phat_hash_before:
        shutil.copy2(backup, main_cpp)
        print("[FAIL] phat_lora.cpp thay đổi ngoài dự kiến!")
        print("[ROLLBACK] Đã khôi phục main.cpp.")
        return 7

    print(f"[PATCH_OK] {TAG}")
    print("[CHANGED] src/main.cpp: thêm LQ backlog rescue 24/8")
    print("[UNCHANGED] src/phat_lora.cpp")
    print(f"[JAM_GUARD] SHA256 sau={phat_hash_after}")
    print("[NEXT] Build rồi flash SU1 trước để test.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
