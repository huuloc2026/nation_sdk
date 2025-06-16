import serial
import struct
import time
import threading
from enum import IntEnum, unique
from typing import Callable, Optional,Tuple

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
        self._inventory_running = False
        self._inventory_thread = None

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
    EPC_UPLOAD = 0x0000    # Notification from reader
    READ_END = 0x0001      # Notification of end

    STOP_INVENTORY = (0x02 << 8) | 0xFF
    STOP_OPERATION = 0x0101

    # RFID Baseband
    CONFIG_BASEBAND = 0x0B00
    QUERY_BASEBAND = 0x0C00
    # Power Control
    CONFIGURE_READER_POWER = 0x0101
    QUERY_READER_POWER = 0x1002
    SET_READER_POWER_CALIBRATION = 0x0103
    # RFID Capability
    QUERY_RFID_ABILITY = (0x05 << 8) | 0x00


class NationReader:
    DEFAULT_PORT = "/dev/ttyUSB0"
    DEFAULT_BAUDRATE = 115200
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
        self.rs485 = False

    def open(self):
        self.uart.open()

    def close(self):
        self.uart.close()

    def send(self, data: bytes):
        self.uart.send(data)

    def receive(self, size: int) -> bytes:
        return self.uart.receive(size)

  



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
        frame_header = b'\x5A'

        mid_value = getattr(mid, 'value', mid)
        category = (mid_value >> 8) & 0xFF
        mid_code = mid_value & 0xFF

        pcw = cls.build_pcw(category, mid_code, rs485=rs485, notify=notify)
        pcw_bytes = pcw.to_bytes(4, 'big')
        addr_bytes = b'\x00' if rs485 else b''
        length_bytes = len(payload).to_bytes(2, 'big')

        frame_content = pcw_bytes + addr_bytes + length_bytes + payload
        crc_bytes = cls.crc16_ccitt(frame_content).to_bytes(2, 'big')

        return frame_header + frame_content + crc_bytes

    @classmethod
    def build_pcw(cls, category: int, mid: int, rs485=False, notify=False) -> int:
        pcw = (PROTO_TYPE << 24) | (PROTO_VER << 16)
        if rs485:
            pcw |= (1 << 13)
        if notify:
            pcw |= (1 << 12)
        pcw |= (category << 8) | mid
        return pcw

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
    
        
    def extract_valid_frames(self, data: bytes) -> list[bytes]:
        """
        Extracts all valid protocol frames from raw byte stream based on:
        - Header = 0x5A
        - Length field (2 bytes)
        - CRC16-CCITT check
        """
        frames = []
        i = 0
        while i < len(data):
            if data[i] != 0x5A:
                i += 1
                continue

            # Minimum valid length
            if i + 9 > len(data):
                break

            length = int.from_bytes(data[i+5:i+7], 'big')
            addr_len = 1 if self.rs485 else 0
            full_len = 1 + 4 + addr_len + 2 + length + 2

            if i + full_len > len(data):
                break

            frame = data[i:i + full_len]
            crc_calc = self.crc16_ccitt(frame[1:-2])
            crc_recv = int.from_bytes(frame[-2:], 'big')

            if crc_calc == crc_recv:
                frames.append(frame)
            else:
                print(f"⚠️ CRC mismatch at index {i}: expected={hex(crc_calc)}, got={hex(crc_recv)}")

            i += full_len

        return frames


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

    def query_rfid_ability(self) -> dict:
        """
        Sends MID=0x00 CAT=0x10 'Query RFID Ability' to get reader capability.
        Corrects for real-world byte order (MAX first, MIN second).
        """
        try:
            self.uart.flush_input()
            frame = self.build_frame(mid=0x1000, payload=b'', rs485=self.rs485)
            self.uart.send(frame)

            time.sleep(0.1)
            raw = self.uart.receive(128)
            if not raw:
                raise Exception("❌ No response from reader.")

            frames = self.extract_valid_frames(raw)
            print(f"📦 Raw frames count: {len(frames)}")

            for idx, frame in enumerate(frames):
                print(f"📦 Frame[{idx}]: {frame.hex()}")
                pcw = int.from_bytes(frame[1:5], 'big')
                mid = pcw & 0xFF
                cat = (pcw >> 8) & 0xFF
                print(f"🔎 MID: {mid:02X}, CAT: {cat:02X}")
                if cat != 0x10 or mid != 0x00:
                    continue

                offset = 8 if self.rs485 else 7
                payload = frame[offset:-2]
                print(f"🔍 Payload Bytes: {payload.hex()}")

                if len(payload) < 3:
                    raise Exception("❌ Payload too short.")

                
                max_power = payload[0]
                min_power = payload[1]
                antenna_count = payload[2]


                freq_list = []
                protocols = []

                try:
                    freq_list_len = payload[3]
                    freq_list = list(payload[4:4 + freq_list_len])
                    print(f"📡 Frequency List Length: {freq_list_len}, Data: {freq_list}")

                    protocol_offset = 4 + freq_list_len
                    protocol_list_len = payload[protocol_offset]
                    protocols = list(payload[protocol_offset + 1 : protocol_offset + 1 + protocol_list_len])
                    print(f"📚 Protocol List Length: {protocol_list_len}, Data: {protocols}")
                except IndexError:
                    print("ℹ️ Reader reports no frequencies or protocols.")

                return {
                    "min_power_dbm": min_power,
                    "max_power_dbm": max_power,
                    "antenna_count": antenna_count,
                    "frequencies": freq_list,
                    "rfid_protocols": protocols
                }

            raise Exception("❌ No matching response frame for RFID ability.")

        except Exception as e:
            print(f"❌ Error in query_rfid_ability: {e}")
            return {}

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

            # print(f"📥 RAW: {raw.hex()}")
            # print(f"🧩 Frame Data: {frame_data}")


            if frame_data['mid'] != 0x00 or frame_data['category'] != 0x01:
                print("❌ Unexpected MID or Category.")
                return {}

            return self._parse_query_info_data(frame_data['data']) or {}

        except Exception as e:
            print(f"❌ Exception in Query_Reader_Information: {e}")
            return {}

    @staticmethod
    def _parse_query_info_data(data: bytes) -> dict:
        result = {}
        try:
            offset = 0

            # 1. Serial Number (next byte is length)
            if offset + 2 > len(data):
                return result
            sn_length = data[offset + 1]
            serial = data[offset + 2:offset + 2 + sn_length].decode('ascii', errors='ignore')
            result['serial_number'] = serial.strip()
            offset += 2 + sn_length

            # 2. Power-on time (U32)
            if offset + 4 > len(data):
                return result
            result['power_on_time_sec'] = int.from_bytes(data[offset:offset + 4], 'big')
            offset += 4

            # 3. Baseband compile time (tag 0x00, then length, then ASCII string)
            if offset + 2 > len(data):
                return result
            bb_len = data[offset + 1]
            baseband = data[offset + 2:offset + 2 + bb_len].decode('ascii', errors='ignore')
            result['baseband_compile_time'] = baseband.strip()
            offset += 2 + bb_len

            # 4. Optional tags (0x01: app_version, 0x02: os_version, 0x03: app_compile_time)
            optional_tags = {0x01, 0x02, 0x03}
            while offset + 2 <= len(data):
                tag = data[offset]
                length = data[offset + 1]
                value = data[offset + 2:offset + 2 + length]
                offset += 2 + length

                if tag == 0x01 and len(value) == 4:
                    v = int.from_bytes(value, 'big')
                    result['app_version'] = f"V{(v>>24)&0xFF}.{(v>>16)&0xFF}.{(v>>8)&0xFF}.{v&0xFF}"
                elif tag == 0x02:
                    result['os_version'] = value.decode('ascii', errors='ignore').strip()
                elif tag == 0x03:
                    result['app_compile_time'] = value.decode('ascii', errors='ignore').strip()
                else:
                    continue

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
        self._inventory_running = True
        payload = self.build_epc_read_payload()
        frame = self.build_frame(mid=MID.READ_EPC_TAG, payload=payload)
        self.send(frame)
        print("🚀 Inventory started")

        def receive_loop():
            while self._inventory_running:
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

        self._inventory_thread = threading.Thread(target=receive_loop, daemon=True)
        self._inventory_thread.start()


    def stop_inventory(self):
        print("🛑 Gửi lệnh STOP (MID=0xFF)...")

        self.uart.flush_input()
        stop_frame = self.build_frame(mid=MID.STOP_OPERATION, payload=b'')
        print(f"➡️ STOP Frame: {stop_frame.hex()}")
        self.send(stop_frame)

        for attempt in range(10):
            time.sleep(0.2)
            raw = self.receive(256)
            frames = self.extract_valid_frames(raw)
            print(f"📥 [{attempt+1}] Nhận {len(raw)} byte, {len(frames)} frame hợp lệ")

            for idx, f in enumerate(frames):
                try:
                    resp = self.parse_frame(f)
                    mid = resp["mid"]
                    data = resp["data"]

                    if mid == MID.STOP_OPERATION:
                        code = data[0] if data else -1
                        if code == 0x00:
                            print("✅ STOP thành công – Reader ở trạng thái IDLE")
                            return True
                        else:
                            print(f"⚠️ STOP lỗi với mã: {code:#02x}")
                            return False

                    elif mid == MID.READ_END:
                        reason = data[0] if data else -1
                        print(f"✅ Inventory ended. Reason: {reason}")
                        return True  # <== Đây là điểm thay đổi

                    else:
                        print(f"↪️ Bỏ qua MID={mid:#02x}")

                except Exception as e:
                    print(f"❌ Lỗi phân tích frame {idx}: {e}")

        print("❌ Không nhận được phản hồi STOP hoặc END sau 10 lần thử")
        return False


    #NEED TODO:

    # def set_power(self, ant: int, power: int, persist: bool = True) -> bool:
    #     """
    #     Configures the transmit power for a specific antenna port.
    #     :param ant: Antenna number (1–64)
    #     :param power: Transmit power in dBm (0–36)
    #     :param persist: Whether to save the setting across power cycles
    #     :return: True if successful, False otherwise
    #     """
    #     try:
    #         if not (1 <= ant <= 64):
    #             raise ValueError("❌ Antenna number must be 1–64")
    #         if not (0 <= power <= 36):
    #             raise ValueError("❌ Power level must be between 0 and 36 dBm")

    #         print(f"⚙️ Setting Antenna {ant} Power to {power} dBm (persist={persist})")

    #         self.uart.flush_input()

    #         # Build TLV: [PID, LEN, VALUE]
    #         params = bytearray()
    #         params += bytes([ant, 1, power])  # PID = ant (0x01–0x40)
    #         if persist:
    #             params += bytes([0xFF, 1, 1])  # Persistence TLV (save)
    #         else:
    #             params += bytes([0xFF, 1, 0])  # Don't save

    #         frame = self.build_frame(mid=0x01, payload=params)
    #         self.uart.send(frame)

    #         time.sleep(0.1)
    #         raw = self.uart.receive(128)
    #         if not raw:
    #             raise Exception("❌ No response received from reader.")

    #         frames = self.extract_valid_frames(raw)
    #         if not frames:
    #             raise Exception("❌ No valid frames received.")

    #         for idx, f in enumerate(frames):
    #             parsed = self.parse_frame(f)
    #             if parsed["mid"] == 0x01 and parsed["category"] == 0x01:
    #                 data = parsed["data"]
    #                 if len(data) < 1:
    #                     raise Exception("❌ No result byte in response.")
    #                 result = data[0]
    #                 if result == 0:
    #                     print("✅ Power configured successfully.")
    #                     return True
    #                 elif result == 1:
    #                     print("⚠️ Error: Unsupported port parameter.")
    #                 elif result == 2:
    #                     print("⚠️ Error: Unsupported power value.")
    #                 elif result == 3:
    #                     print("⚠️ Error: Failed to save setting.")
    #                 else:
    #                     print(f"⚠️ Unknown response code: {result}")
    #                 return False

    #         raise Exception("❌ No matching configuration response found.")

    #     except Exception as e:
    #         print(f"❌ Error in set_power: {e}")
    #         return False


    def get_power(self, ant: int) -> Optional[int]:
        """
        Query the power of a specific antenna (1-64).
        """
        if not (1 <= ant <= 64):
            raise ValueError("Antenna port must be between 1 and 64")

        self.uart.flush_input()
        frame = self.build_frame(mid=0x02, payload=b'', rs485=self.rs485)
        self.uart.send(frame)

        time.sleep(0.1)
        raw = self.uart.receive(128)
        if not raw:
            print("❌ No response for get_power.")
            return None

        frames = self.extract_valid_frames(raw)
        for frame in frames:
            pcw = int.from_bytes(frame[1:5], 'big')
            mid = pcw & 0xFF
            cat = (pcw >> 8) & 0xFF
            if mid != 0x02:
                continue

            # TLV format
            data_offset = 8 if self.rs485 else 7
            data = frame[data_offset:-2]

            i = 0
            while i + 2 <= len(data):
                pid = data[i]
                length = data[i + 1]
                if i + 2 + length > len(data):
                    break
                value = data[i + 2: i + 2 + length]
                if pid == ant:
                    return value[0]  # power in dBm
                i += 2 + length

        print(f"⚠️ Antenna {ant} not found in get_power response.")
        return None


    def set_power(self, power: int, ant: int = 1, persist: bool = False):
        payload = bytearray([
            ant, 0x01, power  # PID = ant, LEN=1, VALUE
        ])

        # Optional persist flag – only include if True
        if persist:
            payload += bytearray([0xFF, 0x01, 0x01])

        frame = self.build_frame(mid=MID.CONFIGURE_READER_POWER, payload=payload)
        print(f"➡️ Frame: {frame.hex()}")

        self.uart.send(frame)
        time.sleep(0.1)
        raw = self.uart.receive(128)
        print(f"📥 Full response: {raw.hex()}")


        frames = self.extract_valid_frames(raw)
        for f in frames:
            parsed = self.parse_frame(f)
            if parsed["mid"] == 0x01 and parsed["category"] == 0x01:
                data = parsed["data"]
                i = 0
                while i + 2 <= len(data):
                    pid = data[i]
                    length = data[i+1]
                    val = data[i+2]
                    if pid == 0x03 or pid == 0xFF:  # Error codes
                        if val == 0:
                            print("✅ Power configured successfully.")
                            return True
                        else:
                            print(f"❌ Power config failed. Code={val}")
                            return False
                    i += 2 + length

        print("❌ No valid power config response found.")
        return False


    def parse_tlv(self, data: bytes) -> dict[int, bytes]:
        result = {}
        i = 0
        while i + 2 <= len(data):
            pid = data[i]
            length = data[i+1]

            if i + 2 + length > len(data):
                print(f"⚠️ Incomplete TLV at byte {i}: PID=0x{pid:02X}, LEN={length}")
                break

            value = data[i+2:i+2+length]
            result[pid] = value
            i += 2 + length

        return result


    def parse_result_code(data: bytes, expected_pid: int) -> int:
        result = self.parse_tlv(data)
        return result.get(expected_pid, b'\xFF')[0]
        
    def set_beeper(self, mode: int) -> bool:
        return self.send_command(0x60, bytes([mode])) is not None

    def get_beeper(self) -> Optional[int]:
        resp = self.send_command(0x61)
        if resp and len(resp) >= 1:
            return resp[0]
        return None
 
    

    
