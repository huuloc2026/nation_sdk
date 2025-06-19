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
    #STOP
    STOP_INVENTORY = (0x02 << 8) | 0xFF
    STOP_OPERATION = 0xFF
    READ_END = 0x1231


    # RFID Baseband
    CONFIG_BASEBAND = 0x0B00
    QUERY_BASEBAND = 0x0C00
    # Power Control
    CONFIGURE_READER_POWER = 0x0201
    QUERY_READER_POWER = 0x0202
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
        self.uart.flush_input()
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

    
    #✅
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

    #✅
    def query_reader_power(self) -> dict[int, int]:
        """
        Queries the current transmit power settings for all antenna ports.

        :return: A dictionary where keys are antenna IDs (1-64)
                 and values are power levels in dBm, or an empty dict on failure.
        """
        try:
            print("🚀 Sending Query Reader Power command...")
            # Use the consistent MID value for querying RFID powers
            command_frame = self.build_frame(MID.QUERY_READER_POWER, payload=b'', rs485=self.rs485)
            self.uart.flush_input()
            self.uart.send(command_frame)

            time.sleep(0.1) # Give reader time to respond
            raw = self.uart.receive(128) # Receive enough bytes for multiple antenna responses

            if not raw:
                print("❌ No response received from reader.")
                return {}

            frame = self.parse_frame(raw)
            if not frame:
                print("❌ Failed to parse response frame.")
                return {}

            mid = frame["mid"]
            data = frame["data"]

            print(f"🔍 Response MID: 0x{mid:02X}, Data: {data.hex()}")

            # Expected response MID is 0x02 (from QUERY_READER_POWER) [19]
            if mid == (MID.QUERY_READER_POWER & 0xFF):
                power_settings = {}
                offset = 0
                while offset + 2 <= len(data): # Each power entry is 2 bytes (PID + Value)
                    ant_id = data[offset] # PID is antenna ID [19]
                    power_dbm = data[offset + 1] # Value is power in dBm [19]
                    power_settings[ant_id] = power_dbm
                    offset += 2
                print(f"✅ Reader power settings queried: {power_settings}")
                return power_settings
            else:
                print("❌ Unexpected response MID for power query.")
                return {}

        except Exception as e:
            print(f"❌ Exception during power query: {e}")
            return {}

    
    def configure_reader_power(self, antenna_powers: dict[int, int], persistence: Optional[bool] = None) -> bool:
        """
        Configures the transmit power for specified antenna ports.
        :param antenna_powers: A dictionary where keys are antenna IDs (1-64)
                               and values are power levels in dBm (0-36).
        :param persistence: If True, settings are saved after power-down.
                            If False, settings are temporary. If None, uses reader default (typically save).
        :return: True if configuration was successful, False otherwise.
        """
        if not antenna_powers:
            print("❌ No antenna powers provided for configuration.")
            return False
 
        payload_parts = []
        for ant_id, power_dbm in antenna_powers.items():
            if not (1 <= ant_id <= 64):
                print(f"❌ Invalid antenna ID: {ant_id}. Must be between 1 and 64.")
                return False
            if not (0 <= power_dbm <= 36): # Max power is 36dBm [18]
                print(f"❌ Invalid power level for antenna {ant_id}: {power_dbm}dBm. Must be between 0 and 36dBm.")
                return False
            payload_parts.append(ant_id.to_bytes(1, 'big')) # PID for antenna [17]
            payload_parts.append(power_dbm.to_bytes(1, 'big')) # Value for power [17]
 
        if persistence is not None:
            payload_parts.append(b'\xFF') # PID for Parameter persistence [17]
            payload_parts.append((0x01 if persistence else 0x00).to_bytes(1, 'big')) # Value for persistence [17]
 
        full_payload = b''.join(payload_parts)
 
        try:
            self.uart.flush_input()
            print(f"🚀 Sending Configure Reader Power command with payload: {full_payload.hex()}")
            command_frame = self.build_frame(MID.CONFIGURE_READER_POWER, payload=full_payload, rs485=self.rs485)
            self.uart.send(command_frame)
 
            time.sleep(0.1) # Give reader time to respond
            raw = self.uart.receive(64)
 
            if not raw:
                print("❌ No response received from reader.")
                return False
 
            frame = self.parse_frame(raw)
            if not frame:
                print("❌ Failed to parse response frame.")
                return False
 
            mid = frame["mid"]
            data = frame["data"]
 
            print(f"🔍 Response MID: 0x{mid:02X}, Data: {data.hex()}")
 
            # Expected response MID is 0x01 (from CONFIGURE_READER_POWER) [5]
            # Data should contain configuration result (0x00 for success) [19]
            if mid == (MID.CONFIGURE_READER_POWER & 0xFF) and len(data) >= 1 and data == 0x00:
                print("✅ Reader power configured successfully.")
                return True
            else:
                result_code = data if len(data) >= 1 else -1
                error_map = {
                    0x01: "the reader hardware does not support the port parameter", # [19]
                    0x02: "The reader does not support the power parameter",       # [19]
                    0x03: "Save failed"                                          # [19]
                }
                error_msg = error_map.get(result_code, "unknown error")
                print(f"❌ Failed to configure reader power. Result code: 0x{result_code:02X} ({error_msg})")
                return False
 
        except Exception as e:
            print(f"❌ Exception during power configuration: {e}")
            return False
 
    #19/06/2025
    ################################################################################
    #                            INVENTORY HEADER                                  #
    ################################################################################
    def is_inventory_running(self):
        return self._inventory_running

    def start_inventory(self, on_tag=None, on_inventory_end=None):
        """
        Bắt đầu inventory. Gán callback nếu cần:
        - on_tag(tag: dict)
        - on_inventory_end(reason: int)
        """
        self._inventory_running = True
        self._on_tag = on_tag
        self._on_inventory_end = on_inventory_end

        payload = self.build_epc_read_payload()
        frame = self.build_frame(mid=MID.READ_EPC_TAG, payload=payload)
        self.send(frame)
        print("🚀 Inventory started")

        self._inventory_thread = threading.Thread(target=self._receive_inventory_loop)
        self._inventory_thread.start()

    def _receive_inventory_loop(self):
        while self._inventory_running:
            try:
                raw = self.receive(64)
                if not raw:
                    continue

                frame = self.parse_frame(raw)
                mid = frame["mid"]

                if mid == 0x00:  # EPC tag
                    tag = self.parse_epc(frame['data'])
                    if "error" in tag:
                        print(f"⚠️ Parse error: {tag['error']}")
                    else:
                        # print(f"📦 Tag EPC: {tag['epc']}, RSSI: {tag['rssi']}, Ant: {tag['antenna_id']}")
                        if self._on_tag:
                            self._on_tag(tag)

                elif mid in MID.all_read_end_mids():  # MID=0x01/0x21/0x31
                    reason = frame['data'][0] if frame['data'] else None
                    print(f"✅ Inventory ended. Reason: {reason}")
                    if self._on_inventory_end:
                        self._on_inventory_end(reason)
                    self._inventory_running = False
                    break

            except Exception as e:
                # print(f"⚠️ Error in inventory loop: {e}")
                continue


    def stop_inventory(self) -> bool:
        """
        Sends the Stop command (MID=0xFF) to halt RFID operations and confirm idle state.
        Returns True if the reader acknowledges stop or issues a valid 'read end' notification.
        """
        
        # Step 1: Stop any running inventory thread if needed
        self._inventory_running = False
        if hasattr(self, '_inventory_thread') and self._inventory_thread and self._inventory_thread.is_alive():
            self._inventory_thread.join(timeout=1)
            print("🧵 Inventory thread stopped.")

        # Step 2: Clear any unread UART data
        try:
            self.uart.flush_input()
        except Exception as e:
            print(f"⚠️ UART flush failed: {e}")

        # Step 3: Send STOP command frame
        stop_frame = self.build_frame(mid=MID.STOP_INVENTORY, payload=b'')
        self.send(stop_frame)

        # Step 4: Wait for confirmation via response or notification
        for attempt in range(10):
            time.sleep(0.2)
            try:
                raw = self.receive(256)
                frames = self.extract_valid_frames(raw)
                

                for idx, f in enumerate(frames):
                    try:
                        resp = self.parse_frame(f)
                        mid = resp["mid"]
                        data = resp["data"]

                        if mid == MID.STOP_OPERATION:  # MID=0xFF, response
                            result = data[0] if data else -1
                            if result == 0x00:
                                print("✅ Reader responded: STOP successful, now IDLE.")
                                return True
                            else:
                                print(f"⚠️ Reader responded: STOP error code {result:#02x}")
                                return False

                        elif mid in MID.all_read_end_mids():
                            reason = data[0] if data else -1
                            if reason == 1:
                                print("✅ Read end notification: stopped by STOP command.")
                                return True
                            else:
                                print(f"↪️ Read ended with reason code {reason}, not STOP command.")

                        else:
                            print(f"🔍 Unrelated frame, MID={mid:#04x}")
                    except Exception as e:
                        print(f"❌ Frame parse error [{idx}]: {e}")
            except Exception as e:
                print(f"⚠️ UART receive error on attempt {attempt+1}: {e}")

        print("❌ STOP failed: no valid response or reading end notification after 10 attempts.")
        return False

    ################################################################################
    #                            ANT HEADER                                        #
    ################################################################################
    
    def send_ant_config(self, ant_mask: int) -> bool:
        """
        Gửi cấu hình MID=0x07 để bật/tắt antenna hub.
        """
        payload = ant_mask.to_bytes(2, byteorder="little")
        frame = self.build_frame(mid=0x07, payload=payload)
        self.send(frame)
        resp = self.receive(64)
        parsed = self.parse_frame(resp)
        result = parsed["data"][0]
        if result == 0x00:
            print(f"✅ Antenna configuration success. Mask=0x{ant_mask:04X}")
            return True
        else:
            print(f"❌ Antenna config failed. Result=0x{result:02X}")
            return False

    def query_enabled_ant_mask(self) -> int:
        """
        Trả về bitmask các antenna đang bật, từ biến nội bộ hoặc mặc định.
        Nếu chưa được set bằng enable/disable_ant(), thì mặc định là antenna 1 bật.
        """
        return getattr(self, '_enabled_ant_mask', 0x0001)


    def switch_ant(self, ant: int) -> bool:
        """
        Switch to a specific antenna for the next read operation.
        Only affects the next command (e.g., read EPC).
        """
        if not (1 <= ant <= 32):
            raise ValueError("Antenna must be 1–32")
        
        ant_mask = 1 << (ant - 1)
        self._last_switch_ant_mask = ant_mask  # Store internally for next read
        print(f"📡 Switched to antenna {ant} (bitmask=0x{ant_mask:08X})")
        return True


    def enable_ant(self, ant: int) -> bool:
        """
        Enable an extended hub antenna (1–16) via MID=0x07.
        Only works in idle state.
        """
        current_mask = self.query_enabled_ant_mask()  # Optional: if you track current state
        updated_mask = current_mask | (1 << (ant - 1))
        return self.send_ant_config(updated_mask)


    def disable_ant(self, ant: int) -> bool:
        """
        Disable an extended hub antenna (1–16) via MID=0x07.
        Only works in idle state.
        """
        current_mask = self.query_enabled_ant_mask()
        updated_mask = current_mask & ~(1 << (ant - 1))
        return self.send_ant_config(updated_mask)
    
    def get_enabled_ants(self) -> list[int]:
        mask = self.query_enabled_ant_mask()
        return [i+1 for i in range(16) if (mask >> i) & 1]
