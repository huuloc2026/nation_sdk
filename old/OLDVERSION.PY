   # def connect(self):
    #     try:
    #         self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
    #         print(f"📡 Connected to {self.port}@{self.baudrate}")
    #     except Exception as e:
    #         print(f"❌ Serial connection failed: {e}")
    #         raise

    # def close(self):
    #     if self.ser and self.ser.is_open:
    #         self.ser.close()
    #         self.ser = None
    #     print("[DISCONNECTED]")


    # def send_command(self, mid: MID, payload: bytes = b"", desc: str = "", rs485: bool = False) -> bytes:
    #     """
    #     Sends a command to the RFID reader following the UART protocol specification.

    #     Frame = HEADER (0x5A) + PCW (4B) + [ADDR if RS485] + LEN (2B) + PAYLOAD + CRC16 (2B)
    #     - CRC16 is computed over everything *after* 0x5A
    #     - RS485 address byte is excluded for UART
    #     """
    #     frame = self.build_frame(mid, payload, rs485)
    #     print(f"▶ Sending Frame ({desc}): {frame.hex()}")
    #     assert self.ser

    #     self.ser.reset_input_buffer()
    #     self.ser.write(frame)

    #     # Delay to allow response (tunable)
    #     self.ser.timeout = 1.0
    #     time.sleep(0.05)

    #     try:
    #         # Read base header length (5A + PCW + [ADDR] + LEN)
    #         min_header_len = 9 if rs485 else 8
    #         header = self.ser.read(min_header_len)

    #         if len(header) < min_header_len or header[0] != FRAME_HEADER:
    #             print(f"⚠️ Header thiếu or invalid start byte: {header.hex()}")
    #             return b""

    #         length = struct.unpack(">H", header[-2:])[0]
    #         body_crc = self.ser.read(length + 2)
    #         if len(body_crc) < length + 2:
    #             print(f"⚠️ Thiếu body hoặc CRC (need {length+2}, got {len(body_crc)})")
    #             return b""

    #         full_frame = header + body_crc

    #         # Validate CRC
    #         recv_crc = int.from_bytes(full_frame[-2:], 'big')
    #         calc_crc = self.crc16_ccitt(full_frame[1:-2])  # Exclude 0x5A
    #         if recv_crc != calc_crc:
    #             print(f"❌ CRC mismatch: recv=0x{recv_crc:04X}, expected=0x{calc_crc:04X}")
    #             return b""

    #         # Check if it's an error frame (MID == 0x00)
    #         pcw = int.from_bytes(full_frame[1:5], 'big')
    #         mid_value = pcw & 0xFF
    #         category = (pcw >> 8) & 0x0F
    #         notify = (pcw >> 12) & 0x01

    #         if mid_value == 0x00 and notify == 1:
    #             error_type = full_frame[8] if len(full_frame) > 8 else None
    #             print(f"❌ Reader Error Frame Received: MID=0x00, Error Type={error_type}")
    #             return b""

    #         print(f"◀ Received Raw: {full_frame.hex()}")
    #         return full_frame

    #     except Exception as e:
    #         print(f"❌ Exception during UART read: {e}")
    #         return b""



    # def start_inventory(self) -> bool:
    #     print("▶ Starting inventory...")

    #     antenna_mask = b'\x00\x00\x00\x01'
    #     mode         = b'\x01'
    #     payload      = antenna_mask + mode

    #     try:
    #         resp = self.send_command(MID.READ_EPC_TAG, payload, desc="Start EPC Inventory")
    #     except Exception as e:
    #         print(f"❌ Error during send_command: {e}")
    #         return False

    #     if not resp:
    #         print("❌ No response at all")
    #         return False

    #     if resp[0] != 0x5A:
    #         print("⚠️ Got mid-stream data first, inventory is running")
    #         self._inventory_running = True
    #         return True

    #     actual_pcw = int.from_bytes(resp[1:5], 'big')
    #     expected   = (PROTO_TYPE << 24) | (PROTO_VER << 16) | (0x02 << 8) | 0x10

    #     # Cho phép cả trường hợp nhận luôn khung EPC (notify = 1, MID = 0x00)
    #     notify = (actual_pcw >> 12) & 0x01
    #     cat    = (actual_pcw >> 8)  & 0x0F
    #     mid    = actual_pcw & 0xFF

    #     if actual_pcw != expected:
    #         if notify == 1 and cat == 0x02 and mid == 0x00:
    #             print("⚠️ Skipped ACK and got EPC upload – inventory is running")
    #             self._inventory_running = True
    #             return True
    #         print(f"❌ Unexpected PCW: {actual_pcw:08X}")
    #         return False

    #     # ✅ Đọc mã kết quả một cách an toàn dựa trên độ dài thực tế
    #     try:
    #         length = struct.unpack(">H", resp[6:8])[0]
    #         result_code_index = 8 + length - 1  # Byte cuối cùng trước CRC
    #         result_code = resp[result_code_index]
    #     except Exception as e:
    #         print(f"❌ Failed to parse result_code: {e}")
    #         return False

    #     if result_code != 0x00:
    #         print(f"❌ Reader rejected start (code={result_code})")
    #         return False

    #     print("✅ Inventory started successfully.")
    #     self._inventory_running = True
    #     return True

    # def stop_inventory(self):
    #     """
    #     Sends MID=0xFF to safely stop any ongoing inventory before starting a new one.
    #     """
    #     print("🛑 Stopping previous inventory...")
    #     resp = self.send_command(MID.STOP_INVENTORY, b'', desc="Stop Inventory")
    #     if not resp or len(resp) < 14:
    #         print("⚠️  No or invalid response for STOP")
    #         return False
    #     result = resp[-3]
    #     if result == 0x00:
    #         print("✅ Inventory stopped successfully")
    #         return True
    #     else:
    #         print(f"❌ Failed to stop (code={result})")
    #         return False


    # def read_epc(self, callback: Callable[[dict], None]) -> None:
    #     def crc_check(data: bytes) -> bool:
    #         from binascii import crc_hqx
    #         computed_crc = crc_hqx(data[1:-2], 0)  # bỏ 0x5A đầu và CRC cuối
    #         received_crc = int.from_bytes(data[-2:], 'big')
    #         return computed_crc == received_crc

    #     def listen_loop():
    #         print("📡 Listening for EPC notifications...")
    #         while self._inventory_running:
    #             try:
    #                 print("👂 Waiting for data...")
    #                 byte = self.ser.read(1)
    #                 print(f"🔹 Byte = {byte.hex()}")
    #                 # 1. Tìm byte đầu 0x5A
    #                 head = self.ser.read(1)
    #                 if head != b'\x5A':
    #                     continue

    #                 # 2. Đọc PCW (4), kiểm tra RS485
    #                 pcw_bytes = self.ser.read(4)
    #                 pcw = int.from_bytes(pcw_bytes, 'big')
    #                 is_notify = (pcw >> 12) & 0x01
    #                 mid = pcw & 0xFF
    #                 rs485_flag = (pcw >> 13) & 0x01

    #                 addr = self.ser.read(1) if rs485_flag else b''
    #                 len_bytes = self.ser.read(2)
    #                 length = struct.unpack(">H", len_bytes)[0]

    #                 # 3. Đọc body + CRC
    #                 payload_crc = self.ser.read(length + 2)
    #                 if len(payload_crc) < length + 2:
    #                     print("⚠️ Không đủ payload + CRC")
    #                     continue

    #                 frame = b'\x5A' + pcw_bytes + addr + len_bytes + payload_crc
    #                 if not crc_check(frame):
    #                     print("❌ CRC sai")
    #                     continue

    #                 payload = payload_crc[:-2]

    #                 # 4. Phân tích MID
    #                 if mid == 0x00:
    #                     tag_data = self.parse_epc(frame)
    #                     print(f"📦 Tag: {tag_data}")
    #                     callback(tag_data)

    #                 elif mid == 0x01:
    #                     print("📦 Kết thúc Inventory.")

    #                 else:
    #                     print(f"⚠️ Nhận MID không xác định: 0x{mid:02X}")

    #             except Exception as e:
    #                 print(f"⚠️ Lỗi khi nhận dữ liệu: {e}")
    #                 self._inventory_running = False
    #                 time.sleep(0.001)

    #         print("🎧 Thread đọc EPC đã dừng.")

    #     threading.Thread(target=listen_loop, daemon=True).start()

    # def parse_epc(self, raw: bytes) -> dict:
    #     # Skip exact PCW/category checks
    #     payload = raw[9:-2]
    #     data = {}
    #     i = 0

    #     # PC + EPC + Antenna (unchanged)…
    #     pc = int.from_bytes(payload[i:i+2], 'big'); i += 2
    #     epc_len = (pc >> 11) * 2
    #     data['epc']     = payload[i:i+epc_len].hex().upper(); i += epc_len
    #     data['antenna'] = payload[i]; i += 1

    #     # TLV parsing:
    #     while i + 2 <= len(payload):
    #         pid    = payload[i]; length = payload[i+1]; value = payload[i+2:i+2+length]
    #         i += 2 + length

    #         if pid == 0x01:      # RSSI
    #             data['rssi'] = int.from_bytes(value, 'big') - 256

    #         elif pid == 0x07:    # UTC timestamp (variable length)
    #             sec_bytes  = value[0:4]
    #             usec_bytes = value[4:4+(length-4)]
    #             seconds = int.from_bytes(sec_bytes, 'big')
    #             # scale microseconds if only 2 bytes:
    #             if len(usec_bytes) == 2:
    #                 usec = int.from_bytes(usec_bytes, 'big') * 1000
    #             else:
    #                 usec = int.from_bytes(usec_bytes, 'big')
    #             data['time'] = f"{seconds}.{usec:06d}"

    #         else:
    #             # handle other PIDs as before…
    #             pass

    #     return data




    # def set_profile(self, profile: int) -> bool:
    #     """
    #     Sets the EPC baseband profile (speed mode) of the reader.
    #     Profile: 0 (slow) to 4 (fast)
    #     Returns True if configuration succeeded, False otherwise.
    #     """
    #     if not (0 <= profile <= 4):
    #         raise ValueError("Profile must be between 0 and 4")

    #     pid = 0x00
    #     value_bytes = bytes([profile, 0x00])  # baseband speed + reserved
    #     length_bytes = struct.pack('>H', len(value_bytes))  # 2 bytes length
    #     payload = bytes([pid]) + length_bytes + value_bytes

    #     print(f"⚙️ Setting baseband profile to {profile}...")
    #     resp = self.send_command(MID.CONFIG_BASEBAND, payload, desc=f"Set Profile {profile}")
    #     return bool(resp)

    

    # def get_profile(self) -> Optional[dict]:
    #     """
    #     Query and parse RFID reader profile. Handles decode errors gracefully.
    #     """
    #     print("🔍 Querying reader profile info...")
    #     resp = self.send_command(MID.QUERY_INFO, b'', desc="Query Reader Info")

    #     if not resp or len(resp) < 14 or resp[0] != FRAME_HEADER:
    #         print("❌ Invalid response.")
    #         return None

    #     try:
    #         data_len = int.from_bytes(resp[6:8], 'big')
    #         data = resp[8:8 + data_len]
    #         i = 0

    #         # Serial Number (null-terminated ASCII)
    #         serial_end = data.find(b'\x00', i)
    #         serial_number = data[i:serial_end].decode('ascii')
    #         i = serial_end + 1

    #         # Power-on time (4 bytes)
    #         power_on_time = int.from_bytes(data[i:i + 4], 'big')
    #         i += 4

    #         # Baseband compile time (null-terminated ASCII)
    #         baseband_end = data.find(b'\x00', i)
    #         baseband_compile_time = data[i:baseband_end].decode('ascii')
    #         i = baseband_end + 1

    #         # App version (4 bytes, convert to dotted string)
    #         app_version_raw = int.from_bytes(data[i:i + 4], 'big')
    #         app_version = f"V{(app_version_raw >> 24) & 0xFF}.{(app_version_raw >> 16) & 0xFF}.{(app_version_raw >> 8) & 0xFF}.{app_version_raw & 0xFF}"
    #         i += 4

    #         # Remaining: OS Version and Compile Time (may be corrupted)
    #         try:
    #             os_end = data.find(b'\x00', i)
    #             os_version = data[i:os_end].decode('ascii')
    #             i = os_end + 1
    #         except Exception:
    #             os_version = "<unreadable>"

    #         try:
    #             app_compile_time = data[i:].decode('ascii', errors='replace')
    #         except Exception:
    #             app_compile_time = data[i:].hex().upper()

    #         profile = {
    #             "serial_number": serial_number,
    #             "power_on_time_sec": power_on_time,
    #             "baseband_compile_time": baseband_compile_time,
    #             "app_version": app_version,
    #             "os_version": os_version,
    #             "app_compile_time": app_compile_time,
    #         }

    #         print("✅ Reader Profile Info:")
    #         for k, v in profile.items():
    #             print(f"  - {k}: {v}")
    #         return profile

    #     except Exception as e:
    #         print(f"⚠️ Failed to parse profile: {e}")
    #         return None

    # def get_ability(self) -> dict:
    #     """
    #     Queries RFID-related capabilities using MID=0x0500.
    #     If the reader does not support this, gracefully falls back to get_power().
    #     """
    #     print("🔍 Querying RFID ability...")
    #     try:
    #         resp = self.send_command(MID.QUERY_RFID_ABILITY, b'', desc="Query RFID Ability")

    #         if not resp or len(resp) < 14 or resp[0] != FRAME_HEADER:
    #             raise ValueError("Empty or malformed response")

    #         # Validate PCW: TYPE=0, VER=1, CAT=0x05, MID=0x00 → PCW = 0x00010500

    #         actual_pcw &= ~(1 << 13)

    #         actual_pcw = int.from_bytes(resp[1:5], 'big')
    #         expected_pcw = (PROTO_TYPE << 24) | (PROTO_VER << 16) | (0x05 << 8) | 0x00
    #         if actual_pcw != expected_pcw:
    #             raise ValueError(f"Unexpected PCW: expected 0x{expected_pcw:08X}, got 0x{actual_pcw:08X}")

    #         data_len = int.from_bytes(resp[6:8], 'big')
    #         data = resp[8:8 + data_len]
    #         i = 0

    #         if len(data) < 4:
    #             raise ValueError("Truncated RFID ability payload")

    #         min_power = data[i]; i += 1
    #         max_power = data[i]; i += 1
    #         antenna_count = data[i]; i += 1

    #         freq_band_count = data[i]; i += 1
    #         freq_bands = [data[i + j] for j in range(freq_band_count)]
    #         i += freq_band_count

    #         if i >= len(data):
    #             raise ValueError("Missing protocol list")

    #         proto_count = data[i]; i += 1
    #         rfid_protocols = [data[i + j] for j in range(proto_count)]

    #         freq_band_map = {
    #             0: "China 920–925 MHz",
    #             3: "FCC 902–928 MHz",
    #             4: "ETSI 865–868 MHz"
    #         }

    #         proto_map = {
    #             0: "ISO18000-6C / EPC C1G2",
    #             1: "ISO18000-6B"
    #         }

    #         result = {
    #             "min_power_dbm": min_power,
    #             "max_power_dbm": max_power,
    #             "antenna_count": antenna_count,
    #             "frequency_bands": [freq_band_map.get(code, f"Unknown({code})") for code in freq_bands],
    #             "rfid_protocols": [proto_map.get(code, f"Unknown({code})") for code in rfid_protocols]
    #         }

    #         print("✅ Reader RFID Ability:")
    #         for k, v in result.items():
    #             print(f"  - {k}: {v}")
    #         return result

    #     except Exception as e:
    #         print(f"⚠️ get_ability failed: {e}")
    #         print("🔁 Falling back to get_power()...")

    #         power_map = self.get_power()
    #         fallback_result = {
    #             "min_power_dbm": 0,
    #             "max_power_dbm": 33,
    #             "antenna_count": len(power_map) if power_map else "unknown",
    #             "frequency_bands": ["default"],
    #             "rfid_protocols": ["ISO18000-6C / EPC C1G2"]
    #         }

    #         print("✅ Fallback Reader Ability:")
    #         for k, v in fallback_result.items():
    #             print(f"  - {k}: {v}")
    #         return fallback_result


    # def version_to_string(version):
    #     return f"V{(version >> 24) & 0xFF}.{(version >> 16) & 0xFF}.{(version >> 8) & 0xFF}.{version & 0xFF}"


    # def set_power(self, power_dict: dict, persistent: bool = True) -> bool:
    #     """
    #     Set RF power (0–36 dBm) for antenna ports. Returns True if success.
    #     """
    #     payload = b''
    #     for ant, power in power_dict.items():
    #         if not (1 <= ant <= 64):
    #             raise ValueError("Antenna port must be 1–64")
    #         if not (0 <= power <= 36):
    #             raise ValueError("Power must be 0–36 dBm")
    #         pid = ant  # PID from 0x01 to 0x40
    #         payload += bytes([pid, power])  # Fixed-length param: no length field

    #     # Optional persistence parameter: PID = 0xFF, Value = 0x01 (save), 0x00 (volatile)
    #     payload += bytes([0xFF, 0x01 if persistent else 0x00])

    #     print(f"⚙️ Setting power: {power_dict} (persistent={persistent})")
    #     resp = self.send_command(MID.CONFIG_POWER, payload, desc="Set Power")

    #     # Check: response must be valid and result byte == 0x00 (success)
    #     if resp and resp[0] == 0x5A and len(resp) >= 14:
    #         result = resp[-3]  # U8 result code is usually right before CRC
    #         return result == 0x00
    #     return False

    # def get_power(self) -> Optional[dict]:
    #     """
    #     Query current configured power levels. Handles TLV and fixed PIDs.
    #     Filters garbage values (outside 0–36 dBm) and skips malformed TLVs.
    #     """
    #     print("🔍 Querying antenna power levels...")
    #     resp = self.send_command(MID.QUERY_POWER, b'', desc="Query Power")

    #     if not resp or len(resp) < 14 or resp[0] != FRAME_HEADER:
    #         print("❌ Invalid or empty response.")
    #         return None

    #     # CRC check
    #     recv_crc = int.from_bytes(resp[-2:], 'big')
    #     calc_crc = self.crc16_ccitt(resp[1:-2])
    #     if recv_crc != calc_crc:
    #         print(f"❌ CRC mismatch: expected 0x{recv_crc:04X}, got 0x{calc_crc:04X}")
    #         return None

    #     try:
    #         data_len = int.from_bytes(resp[6:8], 'big')
    #         data = resp[8:8 + data_len]
    #         print(f"🧪 Raw Payload: {data.hex()}")

    #         i = 0
    #         power_map = {}

    #         while i < len(data):
    #             pid = data[i]

    #             # TLV block (0x21) has 2-byte LEN field
    #             if pid == 0x21 and i + 3 <= len(data):
    #                 length = int.from_bytes(data[i+1:i+3], 'little')
    #                 i += 3
    #                 if i + length > len(data):
    #                     print(f"⚠️ Truncated TLV for PID=0x{pid:02X}, skipping block")
    #                     break
    #                 value = data[i:i+length]
    #                 i += length
    #                 print(f"📦 PID=0x21, LEN={length}, VAL={value.hex()}")

    #                 for j in range(0, len(value), 2):
    #                     if j + 1 >= len(value):
    #                         print(f"⚠️ Incomplete antenna-power pair at byte {j}")
    #                         continue
    #                     ant = value[j]
    #                     pwr = value[j+1]
    #                     if 0 <= pwr <= 36:
    #                         power_map[ant] = pwr
    #             elif 0x01 <= pid <= 0x40:
    #                 # Fixed-PID format (1-byte value)
    #                 if i + 1 >= len(data):
    #                     print(f"⚠️ Missing value byte for fixed PID=0x{pid:02X}")
    #                     break
    #                 val = data[i+1]
    #                 i += 2
    #                 if 0 <= val <= 36:
    #                     power_map[pid] = val
    #                 else:
    #                     print(f"⚠️ Ignoring invalid power {val} dBm for antenna {pid}")
    #                 print(f"📦 PID=0x{pid:02X}, VAL={val}")
    #             else:
    #                 print(f"⚠️ Unknown PID=0x{pid:02X}, skipping rest")
    #                 break

    #         if not power_map:
    #             print("❌ No valid antenna power found.")
    #             return None

    #         print("📡 Antenna Power Map:")
    #         for ant in sorted(power_map):
    #             print(f"  - Antenna {ant}: {power_map[ant]} dBm")

    #         return power_map

    #     except Exception as e:
    #         print(f"⚠️ Exception during TLV parsing: {e}")
    #         return None


    # def stop_epc_read(self) -> bool:
    #     self._inventory_running = False
    #     time.sleep(0.1)
    #     return self.send_command(MID.STOP, b'', desc="Stop EPC Read") is not None

    # @staticmethod
    # def read_exactly(ser: serial.Serial, n: int, timeout=1.0) -> bytes:
    #     buf = bytearray()
    #     start_time = time.time()
    #     while len(buf) < n:
    #         if ser.in_waiting:
    #             buf += ser.read(n - len(buf))
    #         if time.time() - start_time > timeout:
    #             break
    #     return bytes(buf)



    # def start_inventory_with_mode(self, mode: int = 0, callback: Optional[Callable[[dict], None]] = None) -> bool:
    #     payload = b'\x00\x00\x00\x01'  # Antenna 1
    #     payload += bytes([mode])
    #     resp = self.send_command(MID.READ_EPC_TAG, payload, desc=f"Start Inventory Mode={mode}")

    #     if not resp or len(resp) < 14:
    #         return False

    #     if mode == 1 and callback:
    #         def listen_loop():
    #             while self._inventory_running:
    #                 raw = self.ser.read(256)
    #                 if raw:
    #                     try:
    #                         tag = self.parse_tag_upload(raw)
    #                         callback(tag)
    #                     except Exception as e:
    #                         print(f"⚠️ Parse error: {e}")
    #                 time.sleep(0.05)

    #         self._inventory_running = True
    #         self._inventory_thread = threading.Thread(target=listen_loop, daemon=True)
    #         self._inventory_thread.start()

    #     return True

    # def stop_inventory_with_mode(self) -> bool:
    #     return self.stop_inventory()

    # def parse_tid_frame(frame: bytes) -> dict:
    #     pcw = int.from_bytes(frame[1:5], 'big')
    #     if not (pcw >> 12) & 0x1 or (pcw & 0xFF) != 0x20:
    #         raise ValueError("Not a 6B tag upload notification frame")
    #     data_len = int.from_bytes(frame[6:8], 'big')
    #     data = frame[8:8+data_len]
    #     if len(data) < 9:
    #         raise ValueError("Too short to contain 6B TID and antenna ID")
    #     result = {"TID": data[:8].hex().upper(), "Antenna": data[8]}
    #     index = 9
    #     while index + 1 <= len(data):
    #         pid = data[index]
    #         index += 1
    #         if pid == 0x01 and index < len(data):
    #             result["RSSI"] = data[index]
    #             break
    #     return result

    # def parse_national_tag_frame(frame: bytes) -> dict:
    #     pcw = int.from_bytes(frame[1:5], 'big')
    #     if not (pcw >> 12) & 0x1 or (pcw & 0xFF) != 0x30:
    #         raise ValueError("Not a national-standard tag upload frame")
    #     data_len = int.from_bytes(frame[6:8], 'big')
    #     data = frame[8:8+data_len]
    #     tag_code_len = int.from_bytes(data[0:2], 'big')
    #     result = {"TagCoding": data[2:2+tag_code_len].hex().upper()}
    #     index = 2 + tag_code_len
    #     while index + 1 <= len(data):
    #         pid = data[index]
    #         index += 1
    #         if pid == 0x01 and index < len(data):
    #             result["RSSI"] = data[index]
    #             index += 1
    #         elif pid == 0x03 and index + 2 <= len(data):
    #             tid_len = int.from_bytes(data[index:index+2], 'big')
    #             index += 2
    #             result["TID"] = data[index:index+tid_len].hex().upper()
    #             index += tid_len
    #         else:
    #             break
    #     return result

    # def parse_tag_upload(frame: bytes) -> dict:
    #     mid = int.from_bytes(frame[1:5], 'big') & 0xFF
    #     if mid == 0x00:
    #         return NationReader.parse_tag(frame)
    #     elif mid == 0x20:
    #         return NationReader.parse_tid_frame(frame)
    #     elif mid == 0x30:
    #         return NationReader.parse_national_tag_frame(frame)
    #     else:
    #         raise ValueError(f"Unsupported MID: 0x{mid:02X}")