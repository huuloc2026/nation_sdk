import serial
import struct
import time
from typing import List, Dict, Any, Callable


class NationReader:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: serial.Serial | None = None
        self.addr = 0x00  # Default reader address

    @staticmethod
    def crc16_ccitt_msb(data: bytes, poly: int = 0x1021, init: int = 0x0000) -> int:
        crc = init
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc


    @staticmethod
    def get_category_for_mid(mid: int) -> int:
        if mid in [0x00, 0x01, 0x02, 0x03, 0x04]:
            return 1  # Query, config
        elif mid in [0x08, 0x10, 0xFF]:
            return 0  # Real-time command
        return 1
        
    @classmethod
    def build_frame(cls, addr: int, mid: int, payload: bytes = b'', rs485: bool = False, notify: bool = False) -> bytes:
        proto_type = 0x00  # UHF RFID reader protocol
        proto_ver = 0x01   # Protocol version 1
        category = cls.get_category_for_mid(mid)

        # Build Protocol Control Word
        pcw = (proto_type << 24) | (proto_ver << 16)
        if rs485:
            pcw |= (1 << 13)
        if notify:
            pcw |= (1 << 12)
        pcw |= (category << 8) | mid
        pcw_bytes = pcw.to_bytes(4, 'big')

        # Optional: addr (only when rs485 is used)
        addr_bytes = b''
        if rs485:
            addr_bytes = addr.to_bytes(1, 'big')

        # Length of data
        len_bytes = struct.pack('>H', len(payload))

        # Assemble payload from Protocol Control Word → Data
        frame_content = pcw_bytes + addr_bytes + len_bytes + payload

        # Compute CRC over everything after header (excluding header itself)
        crc = cls.crc16_ccitt_msb(frame_content)
        crc_bytes = struct.pack('>H', crc)

        # Final Frame: Header + Content + CRC
        return b'\x5A' + frame_content + crc_bytes

    def connect(self):
        if self.port is None:
            ports = get_available_ports()
            if len(ports) == 0:
                raise RuntimeError("❌ Không tìm thấy cổng serial nào.")
            elif len(ports) == 1:
                self.port = ports[0]
                print(f"✅ Tự động chọn cổng: {self.port}")
            else:
                raise RuntimeError(f"❌ Nhiều cổng serial phát hiện: {ports}. Hãy chọn cụ thể.")
        
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.3)
        print(f"📡 Connected to {self.port} @ {self.baudrate}bps")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 Connection closed.")


    def is_alive(self) -> bool:
        resp = self._send_command(0x00, desc="Ping Reader", expected_mid=0x00)
        return bool(resp)

    def _send_command(self, mid: int, payload: bytes = b'', desc: str = "", expected_mid: int = None) -> bytes:
        frame = self.build_frame(self.addr, mid, payload)
        print(f"[SEND] {desc} → {frame.hex()}")

        self.ser.write(frame)
        time.sleep(0.2)

        response = self.ser.read(128)
        if not response:
            print("❌ No response received.")
            return b''

        print(f"[RECV] {response.hex()}")
        if len(response) < 9:  # Minimum length: header(1) + PCW(4) + len(2) + payload(0) + CRC(2)
            print("❌ Response too short.")
            return b''
        # ✅ Đọc CRC theo MSB-first
        recv_crc = int.from_bytes(response[-2:], 'big')
        calc_crc = self.crc16_ccitt_msb(response[1:-2])
        if recv_crc != calc_crc:
            print(f"⚠️ CRC mismatch: recv={response[-2:].hex()} (→ 0x{recv_crc:04X}), calc={calc_crc:04X}")

        # ✅ Kiểm tra MID
        resp_mid = response[6]
        if expected_mid is not None:
            if resp_mid == 0x06:
                status_code = response[-3]
                if status_code == 0x00:
                    print(f"✅ MID=0x06: Status OK")
                else:
                    print(f"❌ Status MID=0x06: Lỗi với status_code=0x{status_code:02X}")
            elif resp_mid != expected_mid:
                print(f"ℹ️ MID khác kỳ vọng: got 0x{resp_mid:02X}, expect 0x{expected_mid:02X} → vẫn chấp nhận.")

        return response

    def stop(self):
        self._send_command(0xFF, desc="STOP")

    def query_info(self):
        resp = self._send_command(0x00, desc="Query Reader Info", expected_mid=0x00)
        if not resp or len(resp) < 12:
            print("❌ Invalid response.")
            return

        try:
            payload = resp[7:-2]
            idx = 0

            # 1️⃣ Serial Number (ASCII until 0x00 or max 20 bytes)
            end_sn = payload.find(0x00, idx)
            serial_number = payload[idx:end_sn].decode('ascii', errors='ignore')
            idx = end_sn + 1

            # 2️⃣ Power-on Time (4 bytes, U32)
            power_on_time = int.from_bytes(payload[idx:idx + 4], 'big')
            idx += 4

            # 3️⃣ Baseband Compile Time (ASCII until next 0x00)
            end_bb = payload.find(0x00, idx)
            baseband_time = payload[idx:end_bb].decode('ascii', errors='ignore')
            idx = end_bb + 1

            # 4️⃣ Application Software Version (PID = 0x01, 4 bytes)
            if payload[idx] == 0x01:
                app_ver_raw = int.from_bytes(payload[idx + 1:idx + 5], 'big')
                v = [(app_ver_raw >> s) & 0xFF for s in (24, 16, 8, 0)]
                app_version = f"V{v[0]}.{v[1]}.{v[2]}.{v[3]}"
                idx += 5
            else:
                app_version = "N/A"

            # 5️⃣ OS Version (PID = 0x02, variable)
            if idx < len(payload) and payload[idx] == 0x02:
                end_os = payload.find(0x00, idx + 1)
                os_version = payload[idx + 1:end_os].decode('ascii', errors='ignore')
                idx = end_os + 1
            else:
                os_version = "N/A"

            # 6️⃣ App Compile Time (PID = 0x03, variable)
            if idx < len(payload) and payload[idx] == 0x03:
                end_ct = payload.find(0x00, idx + 1)
                compile_time = payload[idx + 1:end_ct].decode('ascii', errors='ignore')
            else:
                compile_time = "N/A"

            # ✅ Show info
            print(f"📟 Serial Number      : {serial_number}")
            print(f"🕒 Power-on Time      : {power_on_time} sec")
            print(f"🛠️  BB Compile Time    : {baseband_time}")
            print(f"🧠 App Version        : {app_version}")
            print(f"🖥️  OS Version         : {os_version}")
            print(f"📅 App Compile Time    : {compile_time}")

        except Exception as e:
            print(f"❌ Parse error in query_info: {e}")

    def query_baseband_version(self):
        resp = self._send_command(0x01, desc="Query Baseband Version", expected_mid=0x01)
        if resp and len(resp) >= 11:
            raw = int.from_bytes(resp[-6:-2], 'big')
            v = [(raw >> s) & 0xFF for s in (24, 16, 8, 0)]
            print(f"✅ Baseband Version: V{v[0]}.{v[1]}.{v[2]}.{v[3]}")

    def query_baudrate(self):
        resp = self._send_command(0x03, desc="Query Baudrate", expected_mid=0x03)
        if resp and len(resp) >= 9:
            code = resp[-3]
            baud = {
                0: "9600", 1: "19200", 2: "115200",
                3: "230400", 4: "460800"
            }.get(code, f"Unknown({code})")
            print(f"✅ Current Baudrate: {baud}")

    def set_baudrate(self, code: int):
        baud_map = {0: "9600", 1: "19200", 2: "115200", 3: "230400", 4: "460800"}
        desc = f"Set Baudrate to {baud_map.get(code, '?')}"
        resp = self._send_command(0x02, bytes([code]), desc=desc, expected_mid=0x02)
        if resp and len(resp) >= 9:
            result = resp[-3]
            if result == 0:
                print("✅ Baudrate set successfully.")
            else:
                print(f"❌ Set baudrate failed with code {result}")

    def query_power(self):
        """
        Truy vấn công suất từng cổng anten hiện tại (tối đa 64).
        """
        resp = self._send_command(0x02, desc="Query Power", expected_mid=0x02)
        if not resp or len(resp) < 9:
            print("❌ Invalid response for power query.")
            return {}

        powers = {}
        payload = resp[7:-2]
        idx = 0
        while idx + 1 < len(payload):
            ant_id = payload[idx]
            power = payload[idx + 1]
            powers[ant_id] = power
            idx += 2

        print("⚡ Power per antenna:")
        for aid, pwr in powers.items():
            print(f"  • Antenna {aid}: {pwr} dBm")
        return powers

    def set_power(self, power_dict: dict[int, int], persistent: bool = True):
        for ant_id, power in power_dict.items():
            if not (1 <= ant_id <= 64):
                raise ValueError(f"Antenna ID {ant_id} out of range (1–64)")
            if not (0 <= power <= 36):
                raise ValueError(f"Power {power} out of range (0–36 dBm)")

        payload = bytearray()
        for ant_id, power in power_dict.items():
            payload.append(ant_id)
            payload.append(power)

        payload.append(0xFF)
        payload.append(0x01 if persistent else 0x00)
 
        return self._send_command(0x01, payload, desc="Set Power", expected_mid=0x01)
        
    def query_ability(self):
        resp = self._send_command(0x00, b'', desc="Query RFID Ability", expected_mid=0x00)
        if resp and len(resp) >= 11:
            min_power = resp[9]
            max_power = resp[10]
            num_ant = resp[11]
            print(f"✅ Min Power: {min_power} dBm, Max Power: {max_power} dBm, Antennas: {num_ant}")
            return {"min": min_power, "max": max_power, "antennas": num_ant}
        else:
            print("❌ Failed to read RFID ability.")
            return None


    def start_inventory(self):
        try:
            # Stop the reader to ensure it's in the Idle state
            stop_command = self.build_frame(self.addr, mid=0xFF)  # Stop command (MID=0xFF)
            self.ser.write(stop_command)
            time.sleep(0.2)

            # Build the start inventory command (MID=0x10)
            # Example parameters: antenna 1 enabled, continuous read mode
            antenna_port = 0x01  # Bitmask for antenna 1
            read_mode = 1  # Continuous read mode

            # Construct the data payload (antenna port + read mode)
            payload = struct.pack('>I', antenna_port) + struct.pack('>B', read_mode)

            # Send the start inventory command
            start_inventory_command = self.build_frame(self.addr, mid=0x10, payload=payload)
            self.ser.write(start_inventory_command)
            print("✅ Inventory started.")

        except Exception as e:
            print(f"❌ Error starting inventory: {e}")

    def stop_inventory(self):
        try:
            # Send the stop command (MID=0xFF)
            stop_command = self.build_frame(self.addr, mid=0xFF)
            self.ser.write(stop_command)
            print("✅ Inventory stopped.")
        except Exception as e:
            print(f"❌ Error stopping inventory: {e}")


    def read_epc_tag(self):
        try:
            # Start the inventory process
            self.start_inventory()

            # Wait for the tag data (notification from reader)
            while True:
                # Read the response (up to 128 bytes in each response)
                response = self.ser.read(128)

                if response:
                    print(f"✅ Received data: {response.hex()}")

                    # Parse EPC data from the response (MID=0x00 for tag data)
                    epc_data = self.parse_inventory_data(response)
                    if epc_data:
                        print(f"🎯 EPC Data: {epc_data}")
                    else:
                        print("❌ No EPC data found.")
                else:
                    print("❌ No response received.")
                
                time.sleep(0.5)  # Adjust timing as needed
        except Exception as e:
            print(f"❌ Error reading EPC tag: {e}")

    def parse_inventory_data(self, data: bytes) -> Dict[str, Any]:
        try:

            # Bỏ qua 7 byte đầu (header và protocol control word)
            epc_start_index = 7
            
            # Giả sử EPC có chiều dài 12 byte (96-bit)
            epc_length = 12  # EPC thường có 12 byte, có thể thay đổi tùy theo cấu hình

            # Lấy dữ liệu EPC từ dữ liệu raw (bắt đầu từ vị trí epc_start_index)
            epc = data[epc_start_index:epc_start_index + epc_length]

            # Chuyển EPC thành chuỗi hex để dễ đọc
            epc_str = epc.hex().upper()

            return {"EPC": epc_str}
        except Exception as e:
            print(f"❌ Lỗi khi phân tích dữ liệu: {e}")
            return {}

    def get_available_ports() -> List[str]:
        """
        Trả về danh sách các cổng serial hiện có (USB, ACM, COM).
        Dùng cho việc gợi ý chọn cổng thiết bị đọc RFID.
        """
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

