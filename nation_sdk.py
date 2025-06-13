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
RESET_CONFIRMATION_CODE = 0x5AA5A55A


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

    def receive(self, size: int) -> bytes:
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
    STOP = 0xFF
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
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self._inventory_running = False

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
    def build_frame(cls, mid: MID, payload: bytes = b"", rs485: bool = False, notify: bool = False) -> bytes:
        """
        Constructs a complete command frame for UART transmission.
        :param mid: Message ID enum (e.g. MID.QUERY_INFO)
        :param payload: Data parameters as bytes
        :param rs485: True to include serial device address (RS485 mode)
        :param notify: True if this is a reader-originated notification
        :return: Byte array representing the complete frame
        """
        # === Frame Header ===
        frame_header = b'\x5A'

        # === Protocol Control Word (PCW) ===
        category = (mid.value >> 8) & 0xFF
        mid_code = mid.value & 0xFF
        pcw = (PROTO_TYPE << 24) | (PROTO_VER << 16)
        if rs485:
            pcw |= (1 << 13)
        if notify:
            pcw |= (1 << 12)
        pcw |= (category << 8) | mid_code
        pcw_bytes = pcw.to_bytes(4, 'big')

        # === Serial device address (optional) ===
        addr_bytes = b'\x00' if rs485 else b''

        # === Data Length ===
        length_bytes = len(payload).to_bytes(2, 'big')  # U16 big-endian

        # === Assemble frame content === (excluding header for CRC)
        frame_content = pcw_bytes + addr_bytes + length_bytes + payload

        # === CRC Check Code (over frame_content only) ===
        crc = cls.crc16_ccitt(frame_content)
        crc_bytes = crc.to_bytes(2, 'big')

        # === Final Frame ===
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

    
    @staticmethod
    def Connect_Reader_And_Initialize(port="/dev/ttyUSB0", baudrate=9600, timeout=0.5) -> bool:
        """
        Initializes the RFID reader by sending a STOP command (MID=0xFF).
        Accepts notification-style response with MID=0x00 if it contains expected STOP echo and status.
        """
        try:
            uart = UARTConnection(port, baudrate, timeout)
            uart.open()
            uart.flush_input()

            print("🚀 Sending STOP command to ensure Idle state...")
            stop_frame = NationReader.build_frame(MID.STOP, payload=b'')
            uart.send(stop_frame)

            time.sleep(0.1)
            raw = uart.receive(64)
            if not raw:
                print("❌ No response received.")
                uart.close()
                return False

            try:
                frame = NationReader.parse_frame(raw)
            except ValueError as e:
                print(f"❌ Frame parsing failed: {e}")
                print(f"📦 Raw bytes: {raw.hex()}")
                uart.close()
                return False

            print(f"🔍 Response MID: 0x{frame['mid']:02X}, Category: 0x{frame['category']:02X}")
            print(f"📦 Full Response: {raw.hex()}")

            # Case 1: direct response (unlikely for STOP, but check just in case)
            if frame["mid"] in [MID.STOP & 0xFF, 0x01] and frame["data_length"] == 1:
                if frame["data"][0] == 0x00:
                    print("✅ Reader successfully initialized and is now in Idle state.")
                    uart.close()
                    return True

            # Case 2: Notification style response (MID=0x00), data = [status, 0x00, 0x00, STOP_MID, 0x00, result]
            if frame["mid"] == 0x00 and frame["data_length"] >= 6:
                returned_mid = frame["data"][3]
                status_code = frame["data"][0]
                result_code = frame["data"][5]

                if returned_mid == MID.STOP & 0xFF and result_code == 0x00:
                    print("✅ Reader responded via notification and is now in Idle state.")
                    uart.close()
                    return True
                else:
                    print(f"❌ Reader returned MID=0x{returned_mid:02X}, status=0x{status_code:02X}, result=0x{result_code:02X}")
                    uart.close()
                    return False

            print("❌ Response did not match any known successful format.")
            uart.close()
            return False

        except Exception as e:
            print(f"❌ Exception during reader init: {e}")
            return False
