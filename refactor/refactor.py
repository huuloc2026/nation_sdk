# nation_reader.py
import serial
import struct
import threading
import time
from enum import IntEnum
from typing import Optional, Callable

# ================= CONSTANTS =================
FRAME_HEADER = 0x5A
PROTO_TYPE = 0x00
PROTO_VER = 0x01
CRC16_CCITT_POLY = 0x1021

class MID(IntEnum):
    QUERY_INFO = 0x0100
    CONFIRM_CONNECTION = 0x12
    READ_EPC_TAG = 0x0210
    STOP_INVENTORY = 0x02FF
    STOP_OPERATION = 0xFF
    CONFIGURE_READER_POWER = 0x0201
    QUERY_READER_POWER = 0x0202
    QUERY_RFID_ABILITY = 0x0500

# ================= UART LAYER =================
class UARTConnection:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.5):
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.lock = threading.Lock()

    def open(self):
        if self.ser and self.ser.is_open:
            return
        self.ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout
        )
        print(f"✅ UART connected: {self.port_name} @ {self.baudrate}bps")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"🔌 UART disconnected: {self.port_name}")

    def send(self, data: bytes):
        if not self.ser or not self.ser.is_open:
            raise IOError("❌ UART not open")
        with self.lock:
            self.ser.write(data)

    def receive(self, size: int = 64) -> bytes:
        if not self.ser or not self.ser.is_open:
            raise IOError("❌ UART not open")
        with self.lock:
            return self.ser.read(size)

    def flush_input(self):
        if self.ser:
            self.ser.reset_input_buffer()

# ================= FRAME HELPERS =================
class FrameHelper:
    @staticmethod
    def crc16_ccitt(data: bytes) -> int:
        crc = 0x0000
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ CRC16_CCITT_POLY) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def build_pcw(category: int, mid: int, rs485=False, notify=False) -> int:
        pcw = (PROTO_TYPE << 24) | (PROTO_VER << 16)
        if rs485:
            pcw |= (1 << 13)
        if notify:
            pcw |= (1 << 12)
        pcw |= (category << 8) | mid
        return pcw

    @classmethod
    def build(cls, mid: int, payload: bytes = b"", rs485: bool = False, notify: bool = False) -> bytes:
        category = (mid >> 8) & 0xFF
        code = mid & 0xFF
        pcw = cls.build_pcw(category, code, rs485, notify).to_bytes(4, 'big')
        addr = b'\x00' if rs485 else b''
        length = len(payload).to_bytes(2, 'big')
        frame = pcw + addr + length + payload
        crc = cls.crc16_ccitt(frame).to_bytes(2, 'big')
        return bytes([FRAME_HEADER]) + frame + crc

    @classmethod
    def parse_frame(cls, raw: bytes) -> dict:
        if len(raw) < 9 or raw[0] != FRAME_HEADER:
            raise ValueError("Invalid frame")
        offset = 1
        pcw = int.from_bytes(raw[offset:offset+4], 'big')
        offset += 4
        rs485_flag = (pcw >> 13) & 0x01
        addr = raw[offset] if rs485_flag else None
        offset += 1 if rs485_flag else 0
        length = int.from_bytes(raw[offset:offset+2], 'big')
        offset += 2
        data = raw[offset:offset+length]
        received_crc = int.from_bytes(raw[offset+length:offset+length+2], 'big')
        calc_crc = cls.crc16_ccitt(raw[1:offset+length])
        if received_crc != calc_crc:
            raise ValueError("CRC mismatch")
        return {
            "mid": pcw & 0xFF,
            "category": (pcw >> 8) & 0xFF,
            "data": data
        }

# ================= MAIN SDK =================
class NationReader:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.5, rs485: bool = False):
        self.uart = UARTConnection(port, baudrate, timeout)
        self.rs485 = rs485
        self._inventory_running = False
        self._inventory_thread = None
        self._on_tag: Optional[Callable] = None
        self._on_inventory_end: Optional[Callable] = None

    def open(self):
        self.uart.open()

    def close(self):
        self.uart.flush_input()
        self.uart.close()

    def _send_and_parse(self, mid, payload=b'', timeout=0.2) -> dict:
        frame = FrameHelper.build(mid, payload, self.rs485)
        self.uart.flush_input()
        self.uart.send(frame)
        time.sleep(timeout)
        raw = self.uart.receive(128)
        return FrameHelper.parse_frame(raw)

    def initialize(self) -> bool:
        try:
            frame = FrameHelper.build(MID.STOP_INVENTORY)
            self.uart.flush_input()
            self.uart.send(frame)
            time.sleep(0.1)
            raw = self.uart.receive(64)
            frame = FrameHelper.parse_frame(raw)
            return frame['mid'] in [0x01, MID.STOP_INVENTORY & 0xFF] and frame['data'][0] == 0x00
        except Exception:
            return False

    def query_power_settings(self):
        try:
            frame = self._send_and_parse(MID.QUERY_READER_POWER)
            data = frame['data']
            return {data[i]: data[i+1] for i in range(0, len(data), 2)}
        except Exception as e:
            print(f"❌ Power query failed: {e}")
            return {}

    def configure_power(self, power_map: dict[int, int], persistent: Optional[bool] = None) -> bool:
        try:
            payload = b''.join(bytes([k, v]) for k, v in power_map.items())
            if persistent is not None:
                payload += b'\xFF' + (b'\x01' if persistent else b'\x00')
            frame = self._send_and_parse(MID.CONFIGURE_READER_POWER, payload)
            return frame['data'][0] == 0x00
        except Exception:
            return False

    def start_inventory(self, on_tag: Callable, on_end: Optional[Callable] = None):
        self._inventory_running = True
        self._on_tag = on_tag
        self._on_inventory_end = on_end
        payload = b'\x00\x00\x00\x01\x01'  # Antenna 1, continuous mode
        frame = FrameHelper.build(MID.READ_EPC_TAG, payload)
        self.uart.send(frame)
        self._inventory_thread = threading.Thread(target=self._inventory_loop)
        self._inventory_thread.start()

    def _inventory_loop(self):
        while self._inventory_running:
            try:
                raw = self.uart.receive(64)
                frame = FrameHelper.parse_frame(raw)
                if frame['mid'] == 0x00 and self._on_tag:
                    tag = frame['data'].hex()
                    self._on_tag({"epc_hex": tag})
                elif frame['mid'] in [0x01, 0x21, 0x31]:
                    reason = frame['data'][0] if frame['data'] else -1
                    if self._on_inventory_end:
                        self._on_inventory_end(reason)
                    self._inventory_running = False
                    break
            except Exception:
                continue

    def stop_inventory(self):
        self._inventory_running = False
        if self._inventory_thread and self._inventory_thread.is_alive():
            self._inventory_thread.join(timeout=1)
        frame = FrameHelper.build(MID.STOP_INVENTORY)
        self.uart.send(frame)
        print("📴 Inventory stopped")
        return True
