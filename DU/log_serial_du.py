#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
log_serial_du.py
Usage:
    python -u log_serial_du.py COM15 output.log
    python -u log_serial_du.py COM15 output.log 115200

- Opens the serial port (default 115200 baud)
- Prints every received line to the terminal
- Writes the same content to the log file immediately
- Ctrl+C stops cleanly
"""

import sys
import time

try:
    import serial
except ImportError:
    print("[ERROR] Thiếu pyserial. Cài bằng:")
    print("  python -m pip install pyserial")
    raise

def main():
    if len(sys.argv) not in (3, 4):
        print("Usage: python -u log_serial_du.py <COM_PORT> <LOG_FILE> [BAUD]")
        print("Example: python -u log_serial_du.py COM15 SU1_COM15_DUAL_CHECK.log")
        return 2

    port = sys.argv[1]
    log_path = sys.argv[2]
    baud = int(sys.argv[3]) if len(sys.argv) == 4 else 115200

    print(f"[LOG] {port} -> {log_path} | BAUD={baud}")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.20,
            write_timeout=1.0,
        )
    except Exception as exc:
        print(f"[ERROR] Không mở được {port}: {exc}")
        return 3

    # Dọn dữ liệu cũ đang nằm trong buffer nhưng không reset board.
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    try:
        with open(log_path, "a", encoding="utf-8", buffering=1) as f:
            while True:
                try:
                    raw = ser.readline()
                except serial.SerialException as exc:
                    print(f"[ERROR] Serial read failed: {exc}")
                    return 4

                if not raw:
                    continue

                text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                print(text, flush=True)
                f.write(text + "\n")
                f.flush()

    except KeyboardInterrupt:
        print("\n[LOG] STOP (Ctrl+C)")
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    raise SystemExit(main())
