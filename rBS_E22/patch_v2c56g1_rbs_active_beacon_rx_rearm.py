#!/usr/bin/env python3
"""Fix the active V2C1 beacon RX re-arm in the supplied rBS gateway.

The earlier V2C56G script edited gui_superframe_beacon_v2b(), but the
supplied gateway defaults to main_v2c1() -> _v2c1_send_beacon().

This script changes ONLY _v2c1_send_beacon():
  * removes the redundant radio.listen() immediately after radio.send();
  * adds RX=AUTO_STM32 | PATCH=V2C5.6G1 to the active beacon log.

Prerequisite: the STM32 must be running firmware equivalent to the supplied
e22_radio(4).c, whose TX_DONE handler calls E22_Radio_StartRxContinuous()
before the event is sent to Pi. The return status of that firmware call is
currently ignored; the marker describes the selected policy, not a hardware
measurement. A successful offline check is not a successful RF bench test.

No SU/DU firmware, RF waveform, power, protocol, ACK policy, slot offsets,
audio codec, AES or watchdog recovery calls are modified.

Run on Pi from /home/pi5/rBS_AIOT:
  python3 patch_v2c56g1_rbs_active_beacon_rx_rearm.py --project-root . --check
  python3 patch_v2c56g1_rbs_active_beacon_rx_rearm.py --project-root .
  python3 patch_v2c56g1_rbs_active_beacon_rx_rearm.py --project-root . --restore

Stop rbs-gateway.service before applying or restoring; start it afterwards.
Works on the uploaded baseline and after the earlier V2C56G patch.
"""

import argparse
import ast
import hashlib
import os
from pathlib import Path
import shutil
import stat
import sys
import tempfile

TAG = "V2C5.6G1_ACTIVE_BEACON_RX_AUTOREARM"
FUNCTION = "_v2c1_send_beacon"
BACKUP_DIR = "backup_before_V2C56G1_ACTIVE_BEACON"
LOG_MARKER = "RX=AUTO_STM32 | PATCH=V2C5.6G1 | "
OLD_BLOCK = (
    "    try:\n"
    "        radio.listen()\n"
    "    except Exception:\n"
    "        pass\n"
)
OLD_LOG = 'f"[rBS V2C1 BEACON] FRAME={frame_id} | TX={tx_ms:.1f}ms | "'
NEW_LOG = OLD_LOG[:-1] + LOG_MARKER + '"'
NEW_BLOCK = (
    "    # " + TAG + "\n"
    "    # STM32 firmware re-arms RX before forwarding EVT_TX_DONE to Pi.\n"
    "    # Do not send a second CMD_START_RX near the first uplink slot.\n"
    "    # RX=AUTO_STM32 reports this policy, not measured RX readiness.\n"
)


def function_node(source):
    tree = ast.parse(source)
    nodes = [n for n in tree.body
             if isinstance(n, ast.FunctionDef) and n.name == FUNCTION]
    if len(nodes) != 1:
        raise ValueError("Expected exactly one " + FUNCTION)
    return tree, nodes[0]


def listen_calls(node):
    return [n for n in ast.walk(node)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and isinstance(n.func.value, ast.Name)
            and n.func.value.id == "radio"
            and n.func.attr == "listen"]


def transform(raw):
    text = raw.decode("utf-8-sig")
    newline = "\r\n" if "\r\n" in text else "\n"
    if newline == "\r\n" and "\n" in text.replace("\r\n", ""):
        raise ValueError("Mixed line endings; inspect this gateway manually")
    source = text.replace("\r\n", "\n")
    tree, node = function_node(source)
    lines = source.splitlines(keepends=True)
    before = "".join(lines[:node.lineno - 1])
    region = "".join(lines[node.lineno - 1:node.end_lineno])
    after = "".join(lines[node.end_lineno:])
    if TAG in region:
        if listen_calls(node) or region.count(LOG_MARKER) != 1:
            raise ValueError("G1 marker exists but active beacon is inconsistent")
        return raw, False
    if TAG in source:
        raise ValueError("G1 marker is outside the active beacon function")
    if len(listen_calls(node)) != 1 or region.count(OLD_BLOCK) != 1:
        raise ValueError("Unexpected active beacon RX block; no changes made")
    if region.count(OLD_LOG) != 1:
        raise ValueError("Unexpected active beacon log; no changes made")
    if region.count("identifier=V2C1_TYPE_BEACON") != 1:
        raise ValueError("Active function does not send the expected V2C1 beacon")
    if region.index(OLD_BLOCK) < region.index("identifier=V2C1_TYPE_BEACON"):
        raise ValueError("RX block precedes beacon send; no changes made")

    # Exact replacement is limited to this one parsed function.
    region = region.replace(OLD_BLOCK, NEW_BLOCK, 1)
    region = region.replace(OLD_LOG, NEW_LOG, 1)
    candidate = before + region + after
    compile(candidate, "rbs_gateway.py", "exec")
    new_tree, new_node = function_node(candidate)
    if listen_calls(new_node):
        raise ValueError("Post-check failed: redundant active listen remains")

    # Every other function, constant and the main entrypoint must be identical.
    unchanged_old = ast.Module(
        body=[n for n in tree.body if n is not node], type_ignores=[])
    unchanged_new = ast.Module(
        body=[n for n in new_tree.body if n is not new_node], type_ignores=[])
    if ast.dump(unchanged_old) != ast.dump(unchanged_new):
        raise ValueError("Post-check failed: unrelated code changed")
    encoded = candidate.replace("\n", newline).encode("utf-8")
    if raw.startswith(b"\xef\xbb\xbf"):
        encoded = b"\xef\xbb\xbf" + encoded
    return encoded, True


def atomic_write(path, data):
    mode = stat.S_IMODE(path.stat().st_mode)
    fd, temporary = tempfile.mkstemp(prefix=".rbs-g1-", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--project-root", required=True, type=Path)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--check", action="store_true", help="Validate without writing")
    group.add_argument("--restore", action="store_true", help="Restore the G1 backup")
    args = parser.parse_args()
    root = args.project_root.expanduser().resolve()
    path = root / "rbs_gateway.py"
    if path.is_symlink() or not path.is_file():
        raise ValueError("Expected a regular rbs_gateway.py in " + str(root))
    backup = root / BACKUP_DIR / "rbs_gateway.py"
    original = path.read_bytes()
    if args.restore:
        saved = backup.read_bytes()
        expected, changed = transform(saved)
        if not changed:
            raise ValueError("Backup already contains G1; cannot restore safely")
        if original == saved:
            print("[OK] Already restored to the pre-G1 file")
            return
        if original != expected:
            raise ValueError("Gateway changed after G1; refusing to overwrite newer edits")
        atomic_write(path, saved)
        print("[OK] Restored rbs_gateway.py from " + str(backup))
        return

    candidate, changed = transform(original)
    print("[TARGET] " + str(path))
    print("[SHA256_BEFORE] " + hashlib.sha256(original).hexdigest())
    if not changed:
        print("[OK] G1 already applied and active beacon structure verified")
        return
    if args.check:
        print("[READY] G1 can patch _v2c1_send_beacon; no files written")
        print("[NOTE] Requires the uploaded STM32 TX_DONE auto-RX firmware")
        return
    backup.parent.mkdir(parents=True, exist_ok=True)
    if backup.exists():
        if backup.read_bytes() != original:
            raise ValueError("Different pre-G1 backup exists; it has been preserved")
    else:
        # Exclusive creation prevents overwriting an earlier backup.
        with backup.open("xb") as handle:
            handle.write(original)
            handle.flush()
            os.fsync(handle.fileno())
        shutil.copystat(path, backup)
    if path.read_bytes() != original:
        raise ValueError("Gateway changed during patch preparation; no replacement made")
    atomic_write(path, candidate)
    print("[OK] Applied " + TAG)
    print("[CHANGED] Active V2C1 beacon RX re-arm and its runtime log only")
    print("[BACKUP] " + str(backup))
    print("[SHA256_AFTER] " + hashlib.sha256(candidate).hexdigest())
    print("[VERIFY] During PTT: [rBS V2C1 BEACON] ... " + LOG_MARKER.rstrip(" |"))
    print("[NEXT] Start rbs-gateway.service and collect simultaneous SU/DU/rBS logs")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, SyntaxError) as exc:
        print("[ERROR] " + str(exc), file=sys.stderr)
        sys.exit(1)
