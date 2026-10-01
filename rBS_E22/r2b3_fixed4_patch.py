#!/usr/bin/env python3
from pathlib import Path
import argparse
import shutil
import sys
import py_compile

PATCH_TAG = "V2C5.3B2R2B3_DUAL_ALTERNATE_ORDER"

HELPER = "\n".join([
    "# ============================================================",
    "# V2C5.3B2-R2B3 - DUAL ALTERNATE ORDER",
    "# PATCH_TAG: V2C5.3B2R2B3_DUAL_ALTERNATE_ORDER",
    "# ============================================================",
    "V2C53B2R2B3_DUAL_ALTERNATE_ORDER = True",
    "",
    "",
    "def _v2c53b2r2b3_alternate_dual_order(st, frame_id, items, meta):",
    "    if not V2C53B2R2B3_DUAL_ALTERNATE_ORDER:",
    "        return False",
    "",
    "    mode = int(st.get(\"mode\", V2C2_MODE_DUAL))",
    "    phase = str(st.get(\"jam_dryrun_phase\", \"STEADY\"))",
    "",
    "    if mode != V2C2_MODE_DUAL:",
    "        return False",
    "    if phase != \"STEADY\":",
    "        return False",
    "",
    "    if len(items) != 2 or len(meta) != 2:",
    "        return False",
    "    if any(m[0] != \"voice\" for m in meta):",
    "        return False",
    "",
    "    pair_ids = [int(m[1]) for m in meta]",
    "    if sorted(pair_ids) != [1, 2]:",
    "        return False",
    "",
    "    by_pair = {}",
    "    for raw, m in zip(items, meta):",
    "        by_pair[int(m[1])] = (raw, m)",
    "",
    "    if (int(frame_id) & 1) == 0:",
    "        order = (1, 2)",
    "        next_pair = 2",
    "    else:",
    "        order = (2, 1)",
    "        next_pair = 1",
    "",
    "    items[:] = [by_pair[order[0]][0], by_pair[order[1]][0]]",
    "    meta[:] = [by_pair[order[0]][1], by_pair[order[1]][1]]",
    "",
    "    print(",
    "        f\"[rBS V2C5.3B2R2B3 ORDER] FRAME={int(frame_id)} | \"",
    "        f\"ORDER=P{order[0]}>P{order[1]} | NEXT_PAIR={next_pair} | \"",
    "        \"ITEMS=2 | EXTRA_PACKET=0 | EXTRA_BYTES=0 | RF_JAM=OFF\"",
    "    )",
    "    return True",
    "",
]) + "\n"


def die(msg):
    print(f"[ERROR] {msg}")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--project-root", required=True)
    args = ap.parse_args()

    root = Path(args.project_root).resolve()
    p = root / "rbs_gateway.py"

    if not p.is_file():
        die(f"Not found: {p}")

    s = p.read_text(encoding="utf-8")

    required = [
        "V2C53B2R2B1_TYPE_NEXT_P1 = 0x2E",
        "V2C53B2R2B1_TYPE_NEXT_P2 = 0x2F",
        "def _v2c53b2r2b1_mark_next_voice",
        "_v2c53b2r2b1_mark_next_voice(",
        "NEXTVOICE_MARKER_PROBE=ON",
        "V2C5_JAM_RF_ENABLE = False",
        "send_burst_raw(items, guard_ms=burst_guard_ms)",
    ]
    missing = [x for x in required if x not in s]
    if missing:
        die("Expected active R2B1 source not found. Missing: " + ", ".join(missing))

    forbidden = [
        "TYPE_DL_PERMISSION",
        "V2C5.3B2R2A",
        "BEACON_INBAND_PERMISSION_PROBE",
    ]
    bad = [x for x in forbidden if x in s]
    if bad:
        die("Unexpected old permission branch present: " + ", ".join(bad))

    # Idempotent check based on source actually inserted.
    if (
        "V2C53B2R2B3_DUAL_ALTERNATE_ORDER = True" in s
        and "def _v2c53b2r2b3_alternate_dual_order" in s
        and "DUAL_ALTERNATE_ORDER=ON" in s
    ):
        print("[OK] V2C5.3B2-R2B3 already applied")
        return

    backup_dir = root / "backup_before_V2C53_B2R2B3"
    backup_dir.mkdir(exist_ok=True)
    backup = backup_dir / "rbs_gateway.py"
    shutil.copy2(p, backup)

    helper_anchor = "\ndef _v2c53b2r2b1_mark_next_voice(st, frame_id, items, meta):\n"
    if s.count(helper_anchor) != 1:
        shutil.copy2(backup, p)
        die(f"R2B1 helper anchor count={s.count(helper_anchor)}")

    s = s.replace(helper_anchor, "\n" + HELPER + helper_anchor, 1)

    call_anchor = (
        "    # V2C5.3B2-R2B1: exact next-VOICE marker; no packet/byte added.\n"
        "    v2c53b2r2b1_count, v2c53b2r2b1_mask = _v2c53b2r2b1_mark_next_voice(\n"
        "        st, frame_open, items, meta\n"
        "    )\n"
    )
    if s.count(call_anchor) != 1:
        shutil.copy2(backup, p)
        die(f"R2B1 pre-send call anchor count={s.count(call_anchor)}")

    replacement = (
        "    # V2C5.3B2-R2B3: alternate clean exact-two-VOICE DUAL order.\n"
        "    # items[] and meta[] always move together.\n"
        "    _v2c53b2r2b3_alternate_dual_order(st, frame_open, items, meta)\n\n"
        + call_anchor
    )
    s = s.replace(call_anchor, replacement, 1)

    banner_anchor = (
        '    print("[rBS V2C5.3B2R2B1] NEXTVOICE_MARKER_PROBE=ON | '
        'TYPE_P1=0x2E | TYPE_P2=0x2F | EXACT_NEXT_ITEM=ON | '
        'EXTRA_PACKET=0 | SU_ACTION=OFF | RF_JAM=OFF")\n'
    )
    if s.count(banner_anchor) != 1:
        shutil.copy2(backup, p)
        die(f"R2B1 startup banner count={s.count(banner_anchor)}")

    r2b3_banner = (
        '    print("[rBS V2C5.3B2R2B3] DUAL_ALTERNATE_ORDER=ON | '
        'EVEN=P1>P2/NEXT_P2 | ODD=P2>P1/NEXT_P1 | '
        'EXACT_2VOICE_ONLY=ON | EXTRA_PACKET=0 | EXTRA_BYTES=0 | RF_JAM=OFF")\n'
    )
    s = s.replace(banner_anchor, banner_anchor + r2b3_banner, 1)

    checks = [
        "V2C53B2R2B3_DUAL_ALTERNATE_ORDER = True",
        "def _v2c53b2r2b3_alternate_dual_order",
        "_v2c53b2r2b3_alternate_dual_order(st, frame_open, items, meta)",
        "DUAL_ALTERNATE_ORDER=ON",
        "EVEN=P1>P2/NEXT_P2",
        "ODD=P2>P1/NEXT_P1",
        "V2C5_JAM_RF_ENABLE = False",
    ]
    missing2 = [x for x in checks if x not in s]
    if missing2:
        shutil.copy2(backup, p)
        die("Post-check failed; restored backup. Missing: " + ", ".join(missing2))

    p.write_text(s, encoding="utf-8")

    try:
        py_compile.compile(str(p), doraise=True)
    except Exception as exc:
        shutil.copy2(backup, p)
        die(f"py_compile failed; rolled back automatically: {exc}")

    print("[OK] Applied rBS V2C5.3B2-R2B3 DUAL alternate order")
    print(f"[OK] Project: {root}")
    print(f"[OK] Backup:  {backup}")
    print("[CHANGED] rbs_gateway.py only")
    print("[DUAL EVEN] P1 -> P2 => marker NEXT Pair2 (0x2F)")
    print("[DUAL ODD ] P2 -> P1 => marker NEXT Pair1 (0x2E)")
    print("[SCOPE] STEADY DUAL + exactly two VOICE items, one per pair")
    print("[AIRTIME] no packet/byte added")
    print("[SAFETY] PREPARE/COMMIT/control behavior unchanged; RF_JAM remains OFF")


if __name__ == "__main__":
    main()
