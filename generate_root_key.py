from __future__ import annotations

import argparse
import secrets
from pathlib import Path


HEADER_TEMPLATE = """#ifndef KHOA_GOC_LOCAL_H
#define KHOA_GOC_LOCAL_H

#include <Arduino.h>

// =====================================================
// VEDC ROOT KEY - PRIVATE LOCAL KEY
//
// TUYET DOI KHONG commit file nay len Git/GitHub.
// SU va DU phai dung CUNG mot root key.
// rBS khong can biet root key vi hien tai chi relay ciphertext.
// =====================================================

#define VEDC_ROOT_KEY_LEN 32

static const uint8_t VEDC_ROOT_KEY[VEDC_ROOT_KEY_LEN] =
{{
{bytes_text}
}};

#endif
"""


def format_key(key: bytes) -> str:
    rows = []
    for i in range(0, len(key), 8):
        chunk = key[i:i + 8]
        rows.append(
            "    " + ", ".join(f"0x{b:02X}" for b in chunk)
            + ("," if i + 8 < len(key) else "")
        )
    return "\n".join(rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate one private 256-bit root key for both SU and DU."
    )
    parser.add_argument(
        "--projects-root",
        default=".",
        help="Folder containing SU/ and DU/ PlatformIO projects.",
    )
    args = parser.parse_args()

    root = Path(args.projects_root).resolve()
    su_src = root / "SU" / "src"
    du_src = root / "DU" / "src"

    if not su_src.is_dir() or not du_src.is_dir():
        raise SystemExit(
            "Khong tim thay SU/src va DU/src. "
            "Hay chay script tu thu muc PlatformIO/Projects "
            "hoac truyen --projects-root."
        )

    key = secrets.token_bytes(32)

    text = HEADER_TEMPLATE.format(
        bytes_text=format_key(key)
    )

    su_path = su_src / "khoa_goc_local.h"
    du_path = du_src / "khoa_goc_local.h"

    su_path.write_text(text, encoding="utf-8")
    du_path.write_text(text, encoding="utf-8")

    print("[OK] Da tao 1 root key 256-bit bang secrets.token_bytes().")
    print(f"[OK] SU: {su_path}")
    print(f"[OK] DU: {du_path}")
    print("[IMPORTANT] Hai file tren giong nhau.")
    print("[IMPORTANT] KHONG upload/commit khoa_goc_local.h len GitHub.")
    print("[IMPORTANT] Hay backup khoa o noi rieng neu can nap lai thiet bi sau nay.")


if __name__ == "__main__":
    main()
