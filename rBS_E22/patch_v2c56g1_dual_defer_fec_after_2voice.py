#!/usr/bin/env python3
'''
V2C56G1-DUAL-AIRTIME-DIAG

Mục tiêu:
- Giữ nguyên baseline G1.
- Trong DUAL, nếu downlink burst đã có >= 2 VOICE thì KHÔNG chèn thêm FEC.
- FEC vẫn nằm pending và chỉ được gửi ở frame khác khi còn ngân sách.
- Không sửa codec/AES/packet format/helper RF.
- Tự backup và tự rollback nếu file sau patch không compile.

Dùng:
  python3 patch_v2c56g1_dual_defer_fec_after_2voice.py --project-root . --check
  python3 patch_v2c56g1_dual_defer_fec_after_2voice.py --project-root .
'''

import argparse
import py_compile
import shutil
import sys
from datetime import datetime
from pathlib import Path

PATCH_TAG = "V2C56G1_DUAL_DEFER_FEC_AFTER_2VOICE"

OLD = '''    gp = int(st.get("grant_pair", 0))
    single_mode = int(st.get("mode", V2C2_MODE_DUAL)) in (V2C2_MODE_SINGLE1, V2C2_MODE_SINGLE2)
    # In a short SINGLE frame, control (especially the other pair's SESSION_START)
    # outranks parity.  FEC is best-effort and remains pending for a later frame.
    allow_fec_now = not (single_mode and bool(st["control_queue"]))
    if allow_fec_now and len(items) < V2C1_MAX_BURST_ITEMS and gp in (1, 2):
        pair = st["pairs"][gp]
        if pair["fec_pending"] is not None:
            raw = pair["fec_pending"]
            items.append(_v2c1_relay_raw(pair, raw))
            meta.append(("fec", gp, None, _v2c1_u32be(raw[4:8])))
'''

NEW = '''    gp = int(st.get("grant_pair", 0))
    single_mode = int(st.get("mode", V2C2_MODE_DUAL)) in (V2C2_MODE_SINGLE1, V2C2_MODE_SINGLE2)

    # V2C56G1_DUAL_DEFER_FEC_AFTER_2VOICE
    # DUAL 300 ms / DL offset 185 ms: hai VOICE LQ da gan het ngan sach airtime.
    # Neu chen FEC sau 2 VOICE, burst co nguy co lan qua beacon ke tiep -> jitter/retry.
    # FEC la best-effort: giu pending de gui o frame khac khi con ngan sach.
    dual_mode_for_fec = int(st.get("mode", V2C2_MODE_DUAL)) == V2C2_MODE_DUAL
    voice_items_before_fec = sum(1 for m in meta if m[0] == "voice")

    # In a short SINGLE frame, control (especially the other pair's SESSION_START)
    # outranks parity. FEC remains pending for a later frame.
    allow_fec_now = not (single_mode and bool(st["control_queue"]))

    dual_fec_defer = bool(dual_mode_for_fec and voice_items_before_fec >= 2)
    if dual_fec_defer:
        allow_fec_now = False
        if gp in (1, 2):
            pair_for_fec = st["pairs"][gp]
            if pair_for_fec["fec_pending"] is not None:
                print(
                    f"[rBS V2C56G1 FEC DEFER] FRAME={frame_open} | MODE=DUAL | "
                    f"VOICE_ITEMS={voice_items_before_fec} | FEC_PAIR=P{gp} | "
                    "REASON=AIRTIME_GUARD"
                )

    if allow_fec_now and len(items) < V2C1_MAX_BURST_ITEMS and gp in (1, 2):
        pair = st["pairs"][gp]
        if pair["fec_pending"] is not None:
            raw = pair["fec_pending"]
            items.append(_v2c1_relay_raw(pair, raw))
            meta.append(("fec", gp, None, _v2c1_u32be(raw[4:8])))
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--project-root",
        default=".",
        help="Thư mục chứa rbs_gateway.py (mặc định: thư mục hiện tại)",
    )
    ap.add_argument(
        "--check",
        action="store_true",
        help="Chỉ kiểm tra baseline/anchor, không sửa file",
    )
    args = ap.parse_args()

    root = Path(args.project_root).expanduser().resolve()
    target = root / "rbs_gateway.py"

    if not target.is_file():
        print(f"[FAIL] Không thấy: {target}")
        return 2

    text = target.read_text(encoding="utf-8")

    required = [
        "V2C5.6G1_ACTIVE_BEACON_RX_AUTOREARM",
        "V2C4_SINGLE_DL_GUARD_MS = 15",
        "V2C2_DUAL_SUPERFRAME_S = 0.300",
        "V2C2_DUAL_DOWNLINK_OFFSET_S = 0.185",
        "def _v2c1_downlink(radio, st):",
    ]
    missing = [x for x in required if x not in text]
    if missing:
        print("[FAIL] Baseline không đúng G1 mong đợi. Thiếu:")
        for x in missing:
            print(f"  - {x}")
        return 3

    if PATCH_TAG in text:
        print(f"[OK] Patch đã có sẵn: {PATCH_TAG}")
        return 0

    count = text.count(OLD)
    if count != 1:
        print(f"[FAIL] Anchor FEC không duy nhất: tìm thấy {count} lần; KHÔNG sửa.")
        return 4

    print("[CHECK] Baseline G1: OK")
    print("[CHECK] FEC anchor: OK, đúng 1 vị trí")
    print("[CHECK] Thay đổi dự kiến: DUAL + >=2 VOICE => defer FEC sang frame khác")

    if args.check:
        print("[CHECK ONLY] Không có file nào bị sửa.")
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = target.with_name(f"{target.name}.bak_before_{PATCH_TAG}_{stamp}")
    shutil.copy2(target, backup)
    print(f"[BACKUP] {backup}")

    patched = text.replace(OLD, NEW, 1)
    target.write_text(patched, encoding="utf-8")

    try:
        py_compile.compile(str(target), doraise=True)
    except Exception as exc:
        shutil.copy2(backup, target)
        print(f"[FAIL] py_compile lỗi: {exc}")
        print("[ROLLBACK] Đã tự khôi phục file gốc.")
        return 5

    verify = target.read_text(encoding="utf-8")
    if PATCH_TAG not in verify:
        shutil.copy2(backup, target)
        print("[FAIL] Không thấy PATCH_TAG sau khi ghi.")
        print("[ROLLBACK] Đã tự khôi phục file gốc.")
        return 6

    print(f"[PATCH_OK] {PATCH_TAG}")
    print("[COMPILE_OK] rbs_gateway.py")
    print("[NOTE] Script KHÔNG tự restart service.")
    print("[NOTE] Hãy restart thủ công rồi kiểm tra log trước khi test DUAL.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
