import struct
import time
from enum import IntEnum
from typing import Callable, List, Optional
from dataclasses import dataclass

# === CRC and Protocol Constants ===
CRC_POLY = 0x1021
CRC_INIT = 0x0000
PROTO_TYPE = 0x00
PROTO_VER = 0x01

# === MID Command Enum ===
class MID(IntEnum):
    QUERY_INFO = 0x0100
    READ_EPC_TAG = 0x0010
    STOP_INVENTORY = 0x00FF
    # Add others as needed


@dataclass
class Tag:
    epc: str
    rssi: int
    antenna: int
    timestamp: float

class RFIDReaderSDK:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = self._open_serial(port, baudrate)
        self.inventory_thread = None
        self.running = False

    @staticmethod
    def crc16_ccitt(data: bytes) -> int:
        crc = 0x0000
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc


    @classmethod
    def build_frame(cls, mid: MID, payload: bytes = b"", rs485: bool = False, notify: bool = False) -> bytes:
        category = (mid.value >> 8) & 0xFF
        mid_code = mid.value & 0xFF
        pcw = (PROTO_TYPE << 24) | (PROTO_VER << 16)
        if rs485:
            pcw |= (1 << 13)
        if notify:
            pcw |= (1 << 12)
        pcw |= (category << 8) | mid_code
        pcw_bytes = pcw.to_bytes(4, 'big')
        addr_bytes = b'\x00' if rs485 else b''
        length_bytes = struct.pack('>H', len(payload))
        frame_content = pcw_bytes + addr_bytes + length_bytes + payload
        crc = cls.crc16_ccitt(frame_content)
        return b'\x5A' + frame_content + crc.to_bytes(2, 'big')

    def _open_serial(self, port, baudrate):
        import serial
        return serial.Serial(port, baudrate, timeout=0.1)

    def send_command(self, mid: int, payload: bytes = b'') -> Optional[bytes]:
        # Build and send protocol frame here
        # Use CRC, Frame Header, PCW, etc.
        ...
        return self._receive_response()

    def _receive_response(self) -> Optional[bytes]:
        # Read and parse response
        ...
        return b''

    def parse_tag(self, data: bytes) -> Tag:
        # Dummy parser
        return Tag(epc="300833B2DDD9014000000000", rssi=-60, antenna=1, timestamp=time.time())

    # === Antenna Management ===
    def switch_ant(self, ant: int) -> bool:
        return self.send_command(0x20, bytes([ant])) is not None

    def enable_ant(self, ant: int) -> bool:
        return self.send_command(0x21, bytes([ant, 0x01])) is not None

    def disable_ant(self, ant: int) -> bool:
        return self.send_command(0x21, bytes([ant, 0x00])) is not None

    # === Tag Reading ===
    def read_single(self, ant: int, timeout: int = 30) -> List[Tag]:
        payload = bytes([ant]) + timeout.to_bytes(2, 'big')
        resp = self.send_command(0x30, payload)
        if not resp:
            return []
        return [self.parse_tag(resp)]

    # === Inventory Management ===
    def start_inventory(self, callback: Callable[[Tag], None]) -> bool:
        if self.running:
            return False
        self.running = True

        def _inventory_loop():
            while self.running:
                resp = self.send_command(0x10)
                if resp:
                    tag = self.parse_tag(resp)
                    callback(tag)
                time.sleep(0.1)

        import threading
        self.inventory_thread = threading.Thread(target=_inventory_loop, daemon=True)
        self.inventory_thread.start()
        return True

    def stop_inventory(self) -> bool:
        self.running = False
        return self.send_command(0xFF) is not None

    def start_inventory_with_mode(self, mode: int, callback: Callable[[Tag], None]) -> bool:
        if self.running:
            return False
        self.running = True

        def _inventory_mode_loop():
            while self.running:
                payload = bytes([mode])
                resp = self.send_command(0x11, payload)
                if resp:
                    tag = self.parse_tag(resp)
                    callback(tag)
                time.sleep(0.1)

        import threading
        self.inventory_thread = threading.Thread(target=_inventory_mode_loop, daemon=True)
        self.inventory_thread.start()
        return True

    def stop_inventory_with_mode(self) -> bool:
        self.running = False
        return self.send_command(0xFE) is not None

    # === Profile Configuration ===
    def set_profile(self, profile: int) -> bool:
        return self.send_command(0x40, bytes([profile])) is not None

    def get_profile(self) -> Optional[int]:
        resp = self.send_command(0x41)
        print(f"🔍 Raw get_profile response: {resp.hex() if resp else 'None'}")
        if resp and len(resp) >= 1:
            return True, resp[0]
        return False, -1

    # === Power Control ===
    def set_power(self, ant: int, power: int, persist: bool = True) -> bool:
        if not (1 <= ant <= 64):
            raise ValueError("Antenna port must be between 1 and 64")
        if not (0 <= power <= 36):
            raise ValueError("Power must be between 0 and 36 dBm")

        payload = bytes([
            ant,      # PID = ant (0x01 - 0x40)
            0x01,     # length
            power     # value
        ])
        
        if persist:
            payload += bytes([
                0xFF,   # PID
                0x01,   # length
                0x01    # save after power down
            ])

        resp = self.send_command(0x01, payload)
        if resp and len(resp) >= 1:
            status = resp[-1]
            if status == 0:
                return True
            print(f"⚠️ Set power failed with code: {status}")
        return False

    def get_power(self, ant: int) -> Optional[int]:
        if not (1 <= ant <= 64):
            raise ValueError("Antenna port must be between 1 and 64")

        resp = self.send_command(0x02)  # No payload for get_power
        if not resp:
            print("❌ No response for get_power.")
            return False, -1

        # Parse TLV response
        i = 0
        while i + 2 <= len(resp):
            pid = resp[i]
            length = resp[i+1]
            if i + 2 + length > len(resp):
                break
            value = resp[i+2:i+2+length]
            if pid == ant:
                return True, value[0]
            i += 2 + length

        print(f"⚠️ Antenna {ant} not found in get_power response.")
        return False, -1



    # === Beeper ===
    def set_beeper(self, mode: int) -> bool:
        return self.send_command(0x60, bytes([mode])) is not None

    def get_beeper(self) -> Optional[int]:
        resp = self.send_command(0x61)
        if resp and len(resp) >= 1:
            return resp[0]
        return None

    # === GPIO (Coming Soon) ===
    def gpio_get(self):
        raise NotImplementedError("GPIO get is coming soon.")

    def gpio_set(self, pin: int, value: int):
        raise NotImplementedError("GPIO set is coming soon.")

    def close(self):
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()

    def _receive_response(self) -> Optional[bytes]:
        try:
            # Read until we get enough data (5A + PCW + LEN + DATA + CRC)
            header = self.serial.read(1)
            if header != b'\x5A':
                return None
            frame = header + self.serial.read(6)  # PCW + LEN = 6 bytes
            if len(frame) < 7:
                return None
            length = int.from_bytes(frame[5:7], 'big')
            data = self.serial.read(length + 2)  # Data + CRC
            return frame + data
        except Exception as e:
            print(f"🧨 Read error: {e}")
            return None