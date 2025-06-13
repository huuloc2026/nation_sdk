import serial
import struct
import time
import threading
from enum import IntEnum, unique
from typing import Callable, Optional

# === Constants ===
CRC16_CCITT_INIT = 0x0000
CRC16_CCITT_POLY = 0x8005


FRAME_HEADER = 0x5A
PROTO_TYPE = 0x00
PROTO_VER = 0x01

RS485_FLAG = 0x00 

READER_NOTIFY_FLAG = 0x00  # Set to 0 for upper computer commands




PROTO_TYPE = 0x00
PROTO_VER = 0x01


class UARTConnection:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.5):
        """
        Initialize the UARTConnection class.
        :param port: Serial port path (e.g. COM3, /dev/ttyUSB0)
        :param baudrate: Baud rate (default: 115200)
        :param timeout: Read timeout in seconds
        """
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.lock = threading.Lock()

    def open(self):
        """
        Open and configure the serial port.
        """
        if self.ser and self.ser.is_open:
            return  # Already open

        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            if not self.ser.is_open:
                self.ser.open()
            print(f"✅ UART Connected to {self.port_name} @ {self.baudrate}bps")
        except serial.SerialException as e:
            raise RuntimeError(f"❌ Failed to open serial port: {e}")

    def close(self):
        """
        Close the serial port if open.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"🔌 UART Disconnected from {self.port_name}")

    def send(self, data: bytes):
        """
        Send raw bytes through UART.
        :param data: Byte array to send
        """
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("❌ UART port is not open")
        with self.lock:
            self.ser.write(data)

    def receive(self, size: int=64) -> bytes:
        """
        Receive a fixed number of bytes from UART.
        :param size: Number of bytes to read
        :return: Bytes read from the port
        """
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("❌ UART port is not open")
        with self.lock:
            data = self.ser.read(size)
        return data
    
    
    def send_raw_bytes(self, frame: bytes):
        if not self.serial_port or not self.serial_port.is_open:
            raise IOError("Serial port not open")
        print(f"➡️  Sending ({len(frame)} bytes):", frame.hex())
        self.serial_port.write(frame)
        self.serial_port.flush()

    def flush_input(self):
        """
        Flush input buffer to discard old data.
        """
        if self.ser:
            self.ser.reset_input_buffer()

    def is_open(self) -> bool:
        return self.ser.is_open if self.ser else False


class MID(IntEnum):
    # Reader Configuration
    QUERY_INFO = 0x0100
    CONFIRM_CONNECTION = 0x12
    # RFID Inventory
    READ_EPC_TAG = (0x02 << 8) | 0x10
    STOP_OPERATION = 0xFF
    EPC_UPLOAD = 0x0000    # Notification from reader
    READ_END = 0x0001      # Notification of end

    STOP_INVENTORY = (0x02 << 8) | 0xFF

    # RFID Baseband
    CONFIG_BASEBAND = 0x0B00
    QUERY_BASEBAND = 0x0C00
    # Power Control
    CONFIG_POWER = 0x0100
    QUERY_POWER = 0x0200
    # RFID Capability
    QUERY_RFID_ABILITY = 0x0500

class NationReader:
    DEFAULT_PORT = "/dev/ttyUSB0"
    DEFAULT_BAUDRATE = 9600
    DEFAULT_TIMEOUT = 0.5

    @classmethod
    def set_uart_defaults(cls, port: str, baudrate: int, timeout: float = 0.5):
        cls.DEFAULT_PORT = port
        cls.DEFAULT_BAUDRATE = baudrate
        cls.DEFAULT_TIMEOUT = timeout

    def __init__(self, port, baudrate,timeout=None):
        self.port = port or NationReader.DEFAULT_PORT
        self.baudrate = baudrate or NationReader.DEFAULT_BAUDRATE
        self.timeout = timeout or NationReader.DEFAULT_TIMEOUT
        self.uart = UARTConnection(self.port, self.baudrate, self.timeout)

    def open(self):
        self.uart.open()

    def close(self):
        self.uart.close()

    def send(self, data: bytes):
        self.uart.send(data)

    def receive(self, size: int) -> bytes:
        return self.uart.receive(size)

    def build_frame(self, mid: int, payload: bytes) -> bytes:
        pcw = (PROTO_TYPE << 24) | (PROTO_VER << 16) | (0x10 << 8) | mid
        frame = bytearray()
        frame.append(FRAME_HEADER)
        frame += pcw.to_bytes(4, 'big')
        frame += len(payload).to_bytes(2, 'big')
        frame += payload
        crc = self.calc_crc(bytes(frame))
        frame += crc.to_bytes(2, 'big')
        return bytes(frame)


    # def parse_frame(self, raw: bytes) -> dict:
    #     if raw[0] != FRAME_HEADER:
    #         raise ValueError("Invalid frame header")
    #     pcw = int.from_bytes(raw[1:5], 'big')
    #     mid = pcw & 0xFF
    #     data_len = int.from_bytes(raw[5:7], 'big')
    #     data = raw[7:7 + data_len]
    #     return {"mid": mid, "data": data}

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


    @staticmethod
    def crc16_bytes(data: bytes) -> bytes:
        """
        Return 2-byte CRC in big-endian order.
        :param data: Input data for CRC computation
        :return: CRC16 result as two-byte big-endian
        """
        crc = crc16_ccitt(data)
        return crc.to_bytes(2, 'big')  # Big-endian per protocol

    @classmethod
    def build_frame(cls, mid, payload: bytes = b"", rs485: bool = False, notify: bool = False) -> bytes:
        """
        Constructs a complete command frame for UART transmission.
        :param mid: Message ID (int or MID enum)
        :param payload: Command payload
        :param rs485: True to include device address field
        :param notify: True if this is a reader-originated notification
        :return: Complete frame as bytes
        """
        frame_header = b'\x5A'

        # Allow either Enum or int for `mid`
        mid_value = getattr(mid, 'value', mid)
        category = (mid_value >> 8) & 0xFF
        mid_code = mid_value & 0xFF

        # === Protocol Control Word (PCW) ===
        pcw = (PROTO_TYPE << 24) | (PROTO_VER << 16)
        if rs485:
            pcw |= (1 << 13)
        if notify:
            pcw |= (1 << 12)
        pcw |= (category << 8) | mid_code
        pcw_bytes = pcw.to_bytes(4, 'big')

        # === Address (RS485 only) ===
        addr_bytes = b'\x00' if rs485 else b''

        # === Length and CRC ===
        length_bytes = len(payload).to_bytes(2, 'big')
        frame_content = pcw_bytes + addr_bytes + length_bytes + payload
        crc_bytes = cls.crc16_ccitt(frame_content).to_bytes(2, 'big')

        return frame_header + frame_content + crc_bytes

    @classmethod
    def parse_frame(cls, raw: bytes) -> dict:
        """
        Parses a received frame from the RFID reader.
        Validates CRC and extracts PCW fields and data.
        :param raw: Full raw frame bytes including header
        :return: Parsed dictionary or raises ValueError
        """
        if len(raw) < 9:
            raise ValueError("Frame too short")

        if raw[0] != FRAME_HEADER:
            raise ValueError("Invalid frame header")

        offset = 1  # skip header

        # === Protocol Control Word ===
        pcw_bytes = raw[offset:offset+4]
        pcw = int.from_bytes(pcw_bytes, 'big')
        offset += 4

        proto_type = (pcw >> 24) & 0xFF
        proto_ver = (pcw >> 16) & 0xFF
        rs485_flag = (pcw >> 13) & 0x01
        notify_flag = (pcw >> 12) & 0x01
        category = (pcw >> 8) & 0xFF
        mid = pcw & 0xFF

        response_type = "notification" if notify_flag else "response"

        # === Optional Serial Address ===
        if rs485_flag:
            addr = raw[offset]
            offset += 1
        else:
            addr = None

        # === Data Length ===
        data_len = int.from_bytes(raw[offset:offset+2], 'big')
        offset += 2

        if offset + data_len + 2 > len(raw):
            raise ValueError("Frame length mismatch or truncated")

        # === Data ===
        data = raw[offset:offset + data_len]
        offset += data_len

        # === CRC ===
        received_crc = int.from_bytes(raw[offset:offset+2], 'big')
        calculated_crc = cls.crc16_ccitt(raw[1:offset])  # exclude frame header

        if received_crc != calculated_crc:
            raise ValueError(f"CRC mismatch! Got 0x{received_crc:04X}, expected 0x{calculated_crc:04X}")

        return {
            "valid": True,
            "type": response_type,
            "proto_type": proto_type,
            "proto_ver": proto_ver,
            "rs485": bool(rs485_flag),
            "notify": bool(notify_flag),
            "pcw": pcw,
            "category": category,
            "mid": mid,
            "address": addr,
            "data_length": data_len,
            "data": data,
            "crc": received_crc,
            "raw": raw
        }

    

    def Connect_Reader_And_Initialize(self) -> bool:
        try:
            self.uart.flush_input()

            print("🚀 Sending STOP command to ensure Idle state...")
            stop_frame = self.build_frame(MID.STOP_INVENTORY, payload=b'')
            self.uart.send(stop_frame)

            time.sleep(0.1)
            raw = self.uart.receive(64)
            if not raw:
                print("❌ No response received.")
                return False

            frame = self.parse_frame(raw)
            print(f"🔍 MID: {frame['mid']}, Data: {frame['data'].hex()}")

            if frame["mid"] in [0x01, MID.STOP_INVENTORY & 0xFF] and frame["data"][0] == 0x00:
                print("✅ Reader successfully initialized.")
                return True

            print("❌ Invalid STOP response.")
            return False

        except Exception as e:
            print(f"❌ Exception during init: {e}")
            return False


    def calc_crc_ccitt(data: bytes, poly=0x1021, init=0x0000) -> int:
        crc = init
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ poly) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc


    def Query_Reader_Information(self) -> dict:
        """
        Sends MID=0x00 Category=0x01 'Query Reader Information' and parses the response.
        """
        try:
            self.uart.flush_input()

            frame = self.build_frame(MID.QUERY_INFO, payload=b'')
            self.uart.send(frame)

            time.sleep(0.1)
            raw = self.uart.receive(128)
            if not raw:
                print("❌ No response received.")
                return {}

            frame_data = self.parse_frame(raw)
            print(f"📦 Raw: {raw.hex()}")
            print(f"🔍 Parsed MID: {frame_data['mid']:02X}, CAT: {frame_data['category']:02X}")

            if frame_data['mid'] != 0x00 or frame_data['category'] != 0x01:
                print("❌ Unexpected MID or Category.")
                return {}

            return self._parse_query_info_data(frame_data['data']) or {}

        except Exception as e:
            print(f"❌ Exception in Query_Reader_Information: {e}")
            return {}

    @staticmethod
    def _parse_query_info_data(data: bytes) -> dict:
        """
        Parses the payload of the Query Reader Information response.
        """
        result = {}
        try:
            offset = 0

            # Serial Number (fixed 21 ASCII bytes)
            serial = data[offset:offset+21].decode('ascii', errors='ignore')
            result['serial_number'] = serial.strip()
            offset += 21

            # Power-on time (U32)
            if offset + 4 > len(data):
                result['power_on_time_sec'] = None
                return result
            result['power_on_time_sec'] = int.from_bytes(data[offset:offset+4], 'big')
            offset += 4

            # Baseband compile time (ASCII, until optional tag or end)
            optional_tags = {0x01, 0x02, 0x03}
            bb_end = offset
            while bb_end < len(data) and data[bb_end] not in optional_tags:
                bb_end += 1
            baseband_time = data[offset:bb_end].decode('ascii', errors='ignore')
            result['baseband_compile_time'] = baseband_time.strip()
            offset = bb_end

            # Optional parameters
            while offset < len(data):
                tag = data[offset]
                offset += 1
                if tag == 0x01 and offset + 4 <= len(data):
                    version = int.from_bytes(data[offset:offset+4], 'big')
                    result['app_version'] = f"V{(version>>24)&0xFF}.{(version>>16)&0xFF}.{(version>>8)&0xFF}.{version&0xFF}"
                    offset += 4
                elif tag in (0x02, 0x03):
                    start = offset
                    while offset < len(data) and data[offset] not in optional_tags:
                        offset += 1
                    val = data[start:offset].decode('ascii', errors='ignore')
                    key = 'os_version' if tag == 0x02 else 'app_compile_time'
                    result[key] = val.strip()
                else:
                    result[f'unknown_tag_0x{tag:02X}'] = True
                    break

        except Exception as e:
            result['error'] = f"Parsing exception: {e}"

        return result

    @staticmethod
    def build_epc_read_payload(antenna_mask: int = 0x00000001, continuous: bool = True) -> bytes:
        payload = antenna_mask.to_bytes(4, 'big')
        payload += b'\x01' if continuous else b'\x00'
        return payload

    def parse_epc(self, data: bytes) -> dict:
        try:
            epc_len = int.from_bytes(data[0:2], 'big')
            epc = data[2:2 + epc_len].hex().upper()
            pc = data[2 + epc_len:2 + epc_len + 2].hex().upper()
            antenna_id = data[2 + epc_len + 2]
            rssi = None
            if len(data) > 2 + epc_len + 3:
                pid = data[2 + epc_len + 3]
                if pid == 0x01:
                    rssi = data[2 + epc_len + 4]
            return {
                "epc": epc,
                "pc": pc,
                "antenna_id": antenna_id,
                "rssi": rssi
            }
        except Exception as e:
            return {"error": f"Parse error: {e}"}



    def start_inventory(self):
        payload = self.build_epc_read_payload()
        frame = self.build_frame(mid=MID.READ_EPC_TAG, payload=payload)
        self.send(frame)
        print("🚀 Inventory started")

        def receive_loop():
            while True:
                try:
                    raw = self.receive(64)
                    if not raw:
                        continue
                    frame = self.parse_frame(raw)

                    if frame['mid'] == 0x00:
                        tag = self.parse_epc(frame['data'])
                        if "error" in tag:
                            print(f"⚠️ Parse error: {tag['error']}")
                        else:
                            print(f"📦 Tag EPC: {tag['epc']}, RSSI: {tag['rssi']}, Ant: {tag['antenna_id']}")
                    elif frame['mid'] == 0x01:
                        reason = frame['data'][0] if frame['data'] else None
                        print(f"✅ Inventory ended. Reason: {reason}")
                        break
                except Exception as e:
                    # print(f"⚠️ Error during receive: {e}")
                    continue

        threading.Thread(target=receive_loop, daemon=True).start()


        
    def stop_inventory(self):
        frame = self.build_frame(mid=MID.STOP_OPERATION, payload=b'')
        self.send(frame)
        print("🛑 Sending STOP command...")

        # Retry receive until valid frame is parsed
        raw = None
        for attempt in range(5):  # max 5 retries
            try:
                raw = self.receive(64)
                if not raw:
                    time.sleep(0.1)
                    continue
                resp = self.parse_frame(raw)

                if resp.get('mid') == MID.STOP_OPERATION and resp['data'] and resp['data'][0] == 0:
                    print("✅ Inventory stopped successfully")
                    return
                else:
                    print(f"❌ Unexpected response: MID={resp.get('mid')}, Data={resp.get('data')}")
                    return
            except ValueError as ve:
                print(f"⚠️ Frame error on stop: {ve} → Retrying...")
                time.sleep(0.1)
            except Exception as e:
                print(f"❌ Unexpected error: {e}")
                break

        print("❌ Failed to stop inventory after retries")
