#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V2C56G2A - SU QUEUE HEADROOM 128

Mục tiêu:
- Chỉ tăng SU_STREAM_FRAME_QUEUE_DEPTH từ 48 -> 128.
- KHÔNG sửa scheduler TDMA.
- KHÔNG sửa packet/AES/FEC/codec.
- KHÔNG sửa phat_lora.cpp hoặc bất kỳ cơ chế friendly-jamming nào.
- Tự backup main.cpp.
- Tự kiểm tra phat_lora.cpp không thay đổi.
- Có --check để chỉ kiểm tra baseline, chưa sửa.

Dùng:
  python patch_v2c56g2a_su_queue128.py --project-root . --check
  python patch_v2c56g2a_su_queue128.py --project-root .
"""

import argparse
import hashlib
import shutil
import sys
from datetime import datetime
from pathlib import Path

OLD = "#define SU_STREAM_FRAME_QUEUE_DEPTH 48"
NEW = "#define SU_STREAM_FRAME_QUEUE_DEPTH 128"
TAG = "V2C56G2A_SU_QUEUE_HEADROOM_128"


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
    radio_cpp = root / "src" / "phat_lora.cpp"

    if not main_cpp.is_file():
        print(f"[FAIL] Không thấy {main_cpp}")
        return 2
    if not radio_cpp.is_file():
        print(f"[FAIL] Không thấy {radio_cpp}")
        return 3

    s = main_cpp.read_text(encoding="utf-8", errors="strict")

    # Các anchor để tránh patch nhầm project/baseline cũ.
    required = [
        "SU_V2C2_BlockTarget",
        "V2C4_DUAL_P1_OFFSET_MS",
        "V2C4_DUAL_P2_OFFSET_MS",
        "QUEUE_DROP_OLD",
        "SU_STREAM_FRAME_QUEUE_DEPTH",
    ]
    missing = [x for x in required if x not in s]
    if missing:
        print("[FAIL] Baseline SU không đúng bản mong đợi. Thiếu:")
        for x in missing:
            print(f"  - {x}")
        return 4

    radio_hash_before = sha256(radio_cpp)

    if NEW in s:
        print(f"[OK] Queue đã là 128. TAG={TAG}")
        print(f"[JAM_GUARD] phat_lora.cpp SHA256={radio_hash_before}")
        return 0

    count = s.count(OLD)
    if count != 1:
        print(f"[FAIL] Anchor queue 48 không duy nhất: tìm thấy {count} lần.")
        print("[FAIL] KHÔNG sửa file.")
        return 5

    print("[CHECK] Baseline SU: OK")
    print("[CHECK] Queue hiện tại: 48 frames = 960 ms")
    print("[PLAN]  Queue mới:    128 frames = 2560 ms")
    print("[PLAN]  Chỉ sửa src/main.cpp đúng 1 dòng define.")
    print("[JAM_GUARD] src/phat_lora.cpp sẽ KHÔNG bị sửa.")
    print(f"[JAM_GUARD] SHA256 trước={radio_hash_before}")

    if args.check:
        print("[CHECK ONLY] Không có file nào bị sửa.")
        return 0

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = main_cpp.with_name(f"main.cpp.bak_before_{TAG}_{stamp}")
    shutil.copy2(main_cpp, backup)
    print(f"[BACKUP] {backup}")

    patched = s.replace(OLD, NEW, 1)
    main_cpp.write_text(patched, encoding="utf-8")

    verify = main_cpp.read_text(encoding="utf-8", errors="strict")
    if verify.count(NEW) != 1 or OLD in verify:
        shutil.copy2(backup, main_cpp)
        print("[FAIL] Verify queue patch lỗi.")
        print("[ROLLBACK] Đã khôi phục main.cpp.")
        return 6

    radio_hash_after = sha256(radio_cpp)
    if radio_hash_after != radio_hash_before:
        shutil.copy2(backup, main_cpp)
        print("[FAIL] phat_lora.cpp thay đổi ngoài dự kiến!")
        print("[ROLLBACK] Đã khôi phục main.cpp.")
        return 7

    print(f"[PATCH_OK] {TAG}")
    print("[CHANGED] src/main.cpp: SU_STREAM_FRAME_QUEUE_DEPTH 48 -> 128")
    print("[UNCHANGED] src/phat_lora.cpp")
    print(f"[JAM_GUARD] SHA256 sau={radio_hash_after}")
    print("[NEXT] Build SU rồi flash từng SU; node_config.h giữ nguyên theo từng Pair.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
