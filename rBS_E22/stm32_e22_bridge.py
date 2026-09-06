"""UART bridge giữa Raspberry Pi và STM32 điều khiển E22-400M30S.

Mục tiêu kiến trúc:
- Raspberry Pi giữ toàn bộ protocol/session/ARQ/FEC/HMI/GPS.
- STM32 chỉ làm driver radio thời gian thực: UART <-> SPI/SX1268.
- E22 được cấu hình cố định cho hệ thống thoại hiện tại:
  433 MHz, BW 500 kHz, SF7, CR 4/5, CRC bật, công suất rBS tối đa 30 dBm.

Giao thức UART nhị phân được chốt ở đây để sau này viết firmware STM32/CubeMX/Keil
khớp 1:1. STM32 chưa cần xử lý logic ứng dụng.

Khung UART (little-endian):
    MAGIC[2] = A5 5A
    VERSION  = 01
    TYPE     = 1 byte
    SEQ      = uint16
    LENGTH   = uint16
    PAYLOAD  = LENGTH bytes
    CRC16    = uint16 CCITT-FALSE trên VERSION..PAYLOAD

Pi -> STM32:
    0x01 PING
    0x02 CONFIG_RADIO
    0x03 START_RX_CONTINUOUS
    0x04 TX_RAW_PACKET
    0x05 RADIO_RESET

STM32 -> Pi:
    0x80 ACK
    0x81 RX_PACKET
    0x82 TX_DONE
    0x83 ERROR
    0x84 STATUS

RX_PACKET payload:
    int16 RSSI_x10_dBm
    int16 SNR_x10_dB
    uint16 RF_LENGTH
    uint8  RF_DATA[RF_LENGTH]

CONFIG_RADIO payload:
    uint32 frequency_hz
    uint32 bandwidth_hz
    uint8  spreading_factor
    uint8  coding_rate_denominator    # 5 => 4/5
    uint8  explicit_header            # 1
    int8   tx_power_dbm               # 30 = mức tối đa của E22-400M30S
    uint16 preamble_symbols           # 8
    uint8  sync_word                  # 0x12, khớp SU/DU hiện tại
    uint8  flags                      # bit0 CRC, bit1 IQ inverted

TX_RAW_PACKET payload là packet LoRa vật lý nguyên vẹn. Adapter send() bên dưới tự
thêm 4-byte RadioHead-compatible header để giữ nguyên protocol rBS cũ.
"""

from __future__ import annotations

import os
import struct
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

try:
    import serial
except ImportError:  # pragma: no cover - phụ thuộc môi trường Raspberry Pi
    serial = None


MAGIC = b"\xA5\x5A"
VERSION = 0x01

CMD_PING = 0x01
CMD_CONFIG_RADIO = 0x02
CMD_START_RX = 0x03
CMD_TX_RAW = 0x04
CMD_RADIO_RESET = 0x05

EVT_ACK = 0x80
EVT_RX_PACKET = 0x81
EVT_TX_DONE = 0x82
EVT_ERROR = 0x83
EVT_STATUS = 0x84

MAX_UART_PAYLOAD = 512


class BridgeError(RuntimeError):
    pass


@dataclass
class _Frame:
    frame_type: int
    seq: int
    payload: bytes


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class STM32E22Bridge:
    """Adapter có API gần giống RFM9x để rbs_gateway.py giữ nguyên logic ứng dụng."""

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: Optional[int] = None,
        *,
        frequency_hz: int = 433_000_000,
        bandwidth_hz: int = 500_000,
        spreading_factor: int = 7,
        coding_rate_denominator: int = 5,
        tx_power_dbm: int = 30,
        preamble_symbols: int = 8,
        sync_word: int = 0x12,
        crc_enabled: bool = True,
        iq_inverted: bool = False,
    ):
        self.port = port or os.environ.get("RBS_STM_UART", "/dev/serial0")
        self.baudrate = int(baudrate or os.environ.get("RBS_STM_BAUD", "115200"))

        self.frequency_hz = int(frequency_hz)
        self.bandwidth_hz = int(bandwidth_hz)
        self.spreading_factor = int(spreading_factor)
        self.coding_rate_denominator = int(coding_rate_denominator)
        self.tx_power_dbm = int(tx_power_dbm)
        self.preamble_symbols = int(preamble_symbols)
        self.sync_word = int(sync_word) & 0xFF
        self.crc_enabled = bool(crc_enabled)
        self.iq_inverted = bool(iq_inverted)

        self.last_rssi: Optional[float] = None
        self.last_snr: Optional[float] = None

        self._ser: Optional[serial.Serial] = None
        self._seq = 0
        self._rx_queue: Deque[tuple[bytes, float, float]] = deque(maxlen=64)
        self._event_queue: Deque[_Frame] = deque(maxlen=128)
        self._rx_buffer = bytearray()

        self.connect()

    # ------------------------------------------------------------------
    # Kết nối / cấu hình
    # ------------------------------------------------------------------
    def connect(self) -> None:
        if serial is None:
            raise BridgeError(
                "Thiếu pyserial. Cài trên Raspberry Pi bằng: sudo apt install python3-serial"
            )
        self.close()
        self._ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=0,
            write_timeout=1.0,
        )
        time.sleep(0.08)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        self._command(CMD_PING, b"", timeout=0.8, expected=(EVT_ACK, EVT_STATUS))
        self.configure_radio()
        self.listen()

    def reconnect(self) -> None:
        self.connect()

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def configure_radio(self) -> None:
        flags = 0
        if self.crc_enabled:
            flags |= 0x01
        if self.iq_inverted:
            flags |= 0x02

        payload = struct.pack(
            "<IIBBBbHBB",
            self.frequency_hz,
            self.bandwidth_hz,
            self.spreading_factor,
            self.coding_rate_denominator,
            1,  # explicit header
            self.tx_power_dbm,
            self.preamble_symbols,
            self.sync_word,
            flags,
        )
        self._command(CMD_CONFIG_RADIO, payload, timeout=1.0, expected=(EVT_ACK,))

    def listen(self) -> None:
        self._command(CMD_START_RX, b"", timeout=0.5, expected=(EVT_ACK,))

    def idle(self) -> None:
        # Không cần ép STM32 ra standby ở tầng ứng dụng. Driver STM tự quản TX/RX.
        # Giữ method này để tương thích gateway cũ.
        return

    # ------------------------------------------------------------------
    # API tương thích RFM9x
    # ------------------------------------------------------------------
    def send(
        self,
        data,
        *,
        destination: int = 0xFF,
        node: int = 0xFF,
        identifier: int = 0,
        flags: int = 0,
    ) -> bool:
        payload = bytes(data)
        raw = bytes(
            (
                destination & 0xFF,
                node & 0xFF,
                identifier & 0xFF,
                flags & 0xFF,
            )
        ) + payload

        if len(raw) > 255:
            raise BridgeError(f"Packet RF quá dài: {len(raw)} byte")

        seq = self._next_seq()
        self._write_frame(CMD_TX_RAW, seq, raw)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            frame = self._read_frame(deadline - time.monotonic())
            if frame is None:
                continue

            if frame.frame_type == EVT_RX_PACKET:
                self._store_rx_event(frame.payload)
                continue

            if frame.frame_type == EVT_TX_DONE and frame.seq == seq:
                return True

            if frame.frame_type == EVT_ERROR and frame.seq == seq:
                text = frame.payload.decode("utf-8", errors="replace")
                raise BridgeError(f"STM32 báo lỗi TX: {text}")

            self._event_queue.append(frame)

        raise BridgeError("Timeout chờ TX_DONE từ STM32")

    def receive(self, *, timeout: float = 0.5, with_header: bool = False):
        if self._rx_queue:
            raw, rssi, snr = self._rx_queue.popleft()
            self.last_rssi = rssi
            self.last_snr = snr
            return raw if with_header else raw[4:]

        deadline = time.monotonic() + max(0.0, float(timeout))
        while time.monotonic() < deadline:
            frame = self._read_frame(deadline - time.monotonic())
            if frame is None:
                continue

            if frame.frame_type == EVT_RX_PACKET:
                item = self._decode_rx_payload(frame.payload)
                if item is None:
                    continue
                raw, rssi, snr = item
                self.last_rssi = rssi
                self.last_snr = snr
                return raw if with_header else raw[4:]

            if frame.frame_type == EVT_ERROR:
                text = frame.payload.decode("utf-8", errors="replace")
                raise BridgeError(f"STM32 báo lỗi radio: {text}")

            self._event_queue.append(frame)

        return None

    # ------------------------------------------------------------------
    # UART framing
    # ------------------------------------------------------------------
    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        if self._seq == 0:
            self._seq = 1
        return self._seq

    def _write_frame(self, frame_type: int, seq: int, payload: bytes) -> None:
        if self._ser is None:
            raise BridgeError("UART bridge chưa mở")
        payload = bytes(payload)
        if len(payload) > MAX_UART_PAYLOAD:
            raise BridgeError(f"UART payload quá dài: {len(payload)}")

        body = struct.pack("<BBHH", VERSION, frame_type & 0xFF, seq & 0xFFFF, len(payload)) + payload
        crc = crc16_ccitt_false(body)
        frame = MAGIC + body + struct.pack("<H", crc)
        self._ser.write(frame)
        self._ser.flush()

    def _command(self, cmd: int, payload: bytes, *, timeout: float, expected) -> _Frame:
        seq = self._next_seq()
        self._write_frame(cmd, seq, payload)
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            frame = self._read_frame(deadline - time.monotonic())
            if frame is None:
                continue

            if frame.frame_type == EVT_RX_PACKET:
                self._store_rx_event(frame.payload)
                continue

            if frame.frame_type == EVT_ERROR and frame.seq == seq:
                text = frame.payload.decode("utf-8", errors="replace")
                raise BridgeError(f"STM32 từ chối lệnh 0x{cmd:02X}: {text}")

            if frame.seq == seq and frame.frame_type in expected:
                return frame

            self._event_queue.append(frame)

        raise BridgeError(f"Timeout lệnh STM32 0x{cmd:02X}")

    def _read_frame(self, timeout: float) -> Optional[_Frame]:
        if self._ser is None:
            raise BridgeError("UART bridge chưa mở")

        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            waiting = self._ser.in_waiting
            if waiting:
                self._rx_buffer.extend(self._ser.read(waiting))
                parsed = self._extract_one_frame()
                if parsed is not None:
                    return parsed
            else:
                parsed = self._extract_one_frame()
                if parsed is not None:
                    return parsed
                time.sleep(0.0005)
        return self._extract_one_frame()

    def _extract_one_frame(self) -> Optional[_Frame]:
        while True:
            if len(self._rx_buffer) < 2:
                return None

            idx = self._rx_buffer.find(MAGIC)
            if idx < 0:
                del self._rx_buffer[:-1]
                return None
            if idx > 0:
                del self._rx_buffer[:idx]

            if len(self._rx_buffer) < 10:
                return None

            version, frame_type, seq, length = struct.unpack_from("<BBHH", self._rx_buffer, 2)
            if version != VERSION or length > MAX_UART_PAYLOAD:
                del self._rx_buffer[0]
                continue

            total = 2 + 6 + length + 2
            if len(self._rx_buffer) < total:
                return None

            body = bytes(self._rx_buffer[2 : 2 + 6 + length])
            crc_rx = struct.unpack_from("<H", self._rx_buffer, 2 + 6 + length)[0]
            del self._rx_buffer[:total]

            if crc16_ccitt_false(body) != crc_rx:
                continue

            payload = body[6:]
            return _Frame(frame_type=frame_type, seq=seq, payload=payload)

    def _decode_rx_payload(self, payload: bytes):
        if len(payload) < 6:
            return None
        rssi_x10, snr_x10, length = struct.unpack_from("<hhH", payload, 0)
        if length > 255 or len(payload) != 6 + length:
            return None
        raw = bytes(payload[6:])
        if len(raw) < 4:
            return None
        return raw, rssi_x10 / 10.0, snr_x10 / 10.0

    def _store_rx_event(self, payload: bytes) -> None:
        item = self._decode_rx_payload(payload)
        if item is not None:
            self._rx_queue.append(item)
