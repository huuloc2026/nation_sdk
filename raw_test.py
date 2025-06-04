# nation_sdk.py

import serial
import struct
import time
from typing import Optional, Dict, Any


class NationReader:
    """
    SDK Nation RFID Reader
    - Dựa trên “RFID Reader Data Communication Protocol”
    - Khung dữ liệu: [0x5A][4-byte Protocol Control Word][(RS485 address)?][2-byte Data Length][N-byte Data][2-byte CRC16]
    - CRC16-CCITT, đa thức X^16 + X^15 + X^2 + 1, init=0, MSB-first.
    """

    FRAME_START = 0x5A

    # Protocol constants (bit offsets trong Protocol Control Word)
    PROTOCOL_TYPE_UHF = 0x00  # 0 → UHF RFID reader protocol (bits 31-24)
    PROTOCOL_VERSION = 0x01   # version 1 (bits 23-16)
    RS485_FLAG = 0            # 0 → không dùng RS485 (bit 13)
    NOTIFY_FLAG = 0           # 0 → lệnh hoặc response, 1 → notification (bit 12)
    CATEGORY_CFG = 0x01       # Category 1: Reader configuration & management / RFID configuration & operation (bits 11-8)
    # Các MID (bits 7-0) sẽ được truyền vào từng method

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.3):
        """
        Khởi tạo NationReader với cổng Serial.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.rs485_addr: Optional[int] = None  # không sử dụng RS485 theo mặc định

    def connect(self) -> None:
        """
        Mở kết nối Serial.
        """
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout
        )
        if not self.ser.is_open:
            self.ser.open()
        print(f"[INFO] Connected to {self.port} @ {self.baudrate}bps")

    def close(self) -> None:
        """
        Đóng kết nối Serial.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] Serial port closed")

    @staticmethod
    def crc16_ccitt_msb(data: bytes, poly: int = 0x1021, init: int = 0x0000) -> int:
        """
        Tính CRC16-CCITT MSB-first (đa thức X^16 + X^15 + X^2 + 1, init=0).
        """
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

    def _build_protocol_control_word(self, mid: int, is_notification: bool = False) -> int:
        """
        Tạo Protocol Control Word (32-bit) theo cấu trúc:
        [31-24] Protocol Type
        [23-16] Version
        [15-14] Reserved (0)
        [13]    RS485 flag
        [12]    Notification flag
        [11-8]  Message Category
        [7-0]   MID
        """
        protocol_type = (self.PROTOCOL_TYPE_UHF & 0xFF) << 24
        version = (self.PROTOCOL_VERSION & 0xFF) << 16
        rs485_flag = (self.RS485_FLAG & 0x01) << 13
        notify_flag = ((1 if is_notification else 0) & 0x01) << 12
        category = (self.CATEGORY_CFG & 0x0F) << 8
        mid_byte = mid & 0xFF
        pcw = protocol_type | version | rs485_flag | notify_flag | category | mid_byte
        return pcw

    def _build_frame(self, mid: int, payload: bytes = b"", is_notification: bool = False) -> bytes:
        """
        Xây dựng khung dữ liệu hoàn chỉnh ([0x5A][PCW][addr?][len][payload][CRC16]) để gửi.
        - Nếu RS485_FLAG=1, chèn thêm 1 byte addr (giá trị self.rs485_addr).
        - Data Length = len(payload) + [các byte (M) nếu có]
        """
        # Bước 1: Header
        frame = bytearray()
        frame.append(self.FRAME_START)

        # Bước 2: Protocol Control Word
        pcw = self._build_protocol_control_word(mid=mid, is_notification=is_notification)
        frame += pcw.to_bytes(4, byteorder='big')

        # Bước 3: Nếu sử dụng RS485 (hiện SDK mặc định RS485_FLAG=0, bỏ qua)
        if self.RS485_FLAG and self.rs485_addr is not None:
            frame.append(self.rs485_addr & 0xFF)

        # Bước 4: Data Length (2 bytes, big-endian) = độ dài payload
        data_len = len(payload)
        frame += data_len.to_bytes(2, byteorder='big')

        # Bước 5: Payload Data
        if payload:
            frame += payload

        # Bước 6: Tính CRC16 cho từ byte thứ 1 đến cuối payload (không kể byte 0 là 0x5A, và không kể 2 byte CRC16 tương lai)
        # Theo tài liệu: “Except for CRC16 data checksum of frame header. ... transmitting data, byte order is big-endian” 
        # CRC được tính trên phần từ byte chỉ số 1 (Protocol Control Word) cho đến byte cuối của payload.
        crc_input = bytes(frame[1:])  # từ PCW đến hết payload
        crc_val = self.crc16_ccitt_msb(crc_input)

        # Bước 7: Chèn CRC16 (2 bytes MSB-first)
        frame += crc_val.to_bytes(2, byteorder='big')

        return bytes(frame)

    def _send_command(self, mid: int, payload: bytes = b"", is_notification: bool = False) -> bytes:
        """
        Gửi khung lệnh và chờ nhận phản hồi (tối đa 128 bytes).
        Trả về raw response (bytes) hoặc b'' nếu không phản hồi.
        """
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port is not open. Call connect() first.")

        frame = self._build_frame(mid=mid, payload=payload, is_notification=is_notification)
        # In hex để debug
        print(f"[SEND] MID=0x{mid:02X} → {frame.hex()}")
        self.ser.write(frame)
        time.sleep(0.05)

        # Đọc tối đa 128 bytes (timeout đã được cấu hình)
        response = self.ser.read(128)
        if not response:
            print("[WARN] No response received.")
            return b""

        print(f"[RECV] {response.hex()}")

        # Kiểm tra CRC16: response[-2:] là CRC nhận được, tính CRC trên response[1:-2]
        recv_crc = int.from_bytes(response[-2:], byteorder='big')
        calc_crc = self.crc16_ccitt_msb(response[1:-2])
        if recv_crc != calc_crc:
            print(f"[ERROR] CRC mismatch: recv=0x{recv_crc:04X}, calc=0x{calc_crc:04X}")

        # Kiểm tra MID: byte thứ 6 trong response (offset từ 0: 0=0x5A, 1-4=PCW, 5-6=addr? hoặc data_len ???)
        # Đơn giản: lấy MID từ PCW (bằng cách lấy PCW cuối cùng & 0xFF)
        # PCW nằm ở response[1..4], do RS485_FLAG=0, nên MID = response[4] & 0xFF
        # response[1:5] = PCW, byte cuối cùng của nó là MID
        resp_mid = response[1 + 3] & 0xFF  # byte index 4
        if resp_mid != mid and resp_mid != 0x00 and resp_mid != 0xFF:
            print(f"[WARN] Unexpected MID: expected=0x{mid:02X}, got=0x{resp_mid:02X}")

        return response

    # ==========================
    # Các lệnh mẫu (MID + payload) theo tài liệu
    # ==========================

    def query_reader_info(self) -> Dict[str, Any]:
        """
        MID = 0x00: Query Reader Information
        Response MID=0x00, trả về:
          - Reader sequential number (ASCII, variable length)
          - Power-on time (U32)
          - Baseband compilation time (ASCII, variable length)
          - Application software version (U32)
          - OS version (ASCII, variable length)
          - Compiled time of application software (ASCII, variable length)
        """
        raw = self._send_command(mid=0x00, payload=b"")
        if not raw:
            return {}

        # Bỏ header 0x5A, PCW, data_len:
        idx = 1 + 4  # 1 byte 0x5A + 4 byte PCW
        # Nếu RS485_FLAG=1 thì thêm 1 byte addr
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx:idx+2], byteorder='big')
        idx += 2

        data = raw[idx: idx + data_len]
        # Giải mã lần lượt:
        cursor = 0
        # Reader sequential number (U8 variable): trước hết đọc 1 byte length? Thực chất table mô tả: (M) U8 variable length ASCII
        # Ở mục này tài liệu cho biết “Reader sequential number (M) U8, variable length ASCII” – nghĩa là đầu tiên 1 byte U8 = độ dài, sau đó ASCII
        seq_len = data[cursor]
        cursor += 1
        seq_num = data[cursor: cursor + seq_len].decode('ascii', errors='ignore')
        cursor += seq_len

        # Power-on time (M) U32
        power_on_time = int.from_bytes(data[cursor: cursor + 4], byteorder='big')
        cursor += 4

        # Baseband compilation time (M) U8 variable length ASCII
        bb_len = data[cursor]
        cursor += 1
        bb_time = data[cursor: cursor + bb_len].decode('ascii', errors='ignore')
        cursor += bb_len

        # Application software version (PID=0x01, U32)
        # Tìm PID=0x01
        app_ver = None
        while cursor < len(data):
            pid = data[cursor]
            cursor += 1
            if pid == 0x01:
                # U32
                app_ver = int.from_bytes(data[cursor: cursor + 4], byteorder='big')
                cursor += 4
                break
            else:
                # Nếu không phải, skip: đầu tiên 2 byte length rồi skip
                length = int.from_bytes(data[cursor: cursor + 2], byteorder='big')
                cursor += 2 + length

        return {
            "serial_number": seq_num,
            "power_on_time": power_on_time,
            "baseband_comp_time": bb_time,
            "app_software_version": app_ver
        }

    def query_baseband_version(self) -> Optional[int]:
        """
        MID = 0x01: Query Baseband Software Version
        Response MID=0x01, (M) U32 4: version, ví dụ 0x00010000 → V1.0.0.0
        """
        raw = self._send_command(mid=0x01, payload=b"")
        if not raw:
            return None

        # Tương tự: bỏ header, lấy data_len, dữ liệu
        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2

        data = raw[idx: idx + data_len]
        # Dữ liệu (M) U32
        version = int.from_bytes(data[0:4], byteorder='big')
        return version

    def set_reader_power(self, antenna_powers: Dict[int, int], save_persistence: bool = True) -> bool:
        """
        MID = 0x01 (Configure the Reader Power): thiết lập công suất cho từng cổng anten
        Payload:
          - Mỗi antenna port i: PID = 0x0(i+1), Data U8 1 byte (0~36 dBm)
          - Cuối cùng thêm parameter persistence (PID=0xFF, U8: 0=power down not save, 1=save)
        Ví dụ: antenna_powers = {1: 20, 2: 18, 3: 15}
        """
        payload = bytearray()
        for ant, pw in antenna_powers.items():
            pid = 0x00 + ant  # anten 1 → PID=0x01, anten 2→PID=0x02, …
            payload.append(pid)
            payload.append(pw & 0xFF)
        # Thêm parameter persistence
        payload.append(0xFF)
        payload.append(1 if save_persistence else 0)

        raw = self._send_command(mid=0x01, payload=bytes(payload))
        if not raw:
            return False

        # Phản hồi: MID=0x01, (M) U8 config result: 0=ok, 1=no support, 2=no support power, 3=save failed
        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2
        result = raw[idx]
        return result == 0

    def query_reader_power(self) -> Dict[int, int]:
        """
        MID = 0x02: Query the Reader Power
        Response MID=0x02, trả về:
          - Mỗi PID=0x01..0x40 (tối đa 64 anten), Data U8 1: công suất (0~36)
          - Cuối cùng (M) U8 config result (0=ok, 1=no support, 2=no power, 3=save failed)
        """
        raw = self._send_command(mid=0x02, payload=b"")
        if not raw:
            return {}

        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2
        data = raw[idx: idx + data_len]
        cursor = 0
        powers = {}
        while cursor < len(data):
            pid = data[cursor]
            cursor += 1
            if pid == 0xFF:
                # Config result cuối cùng, bỏ qua
                break
            # Mỗi antenna: Data U8
            power = data[cursor]
            cursor += 1
            ant_num = pid  # pid=1→antenna1, pid=2→antenna2, …
            powers[ant_num] = power
        return powers

    def set_rf_band(self, band: int, save_persistence: bool = True) -> bool:
        """
        MID = 0x03: Configure the RF frequency band of the Reader
        Payload:
          - (M) U8: band code (0..8)
          - PID=0x01 (persistence) U8: 0=power down not save, 1=power down save
        """
        payload = bytearray([band & 0xFF, 0x01, 1 if save_persistence else 0])
        raw = self._send_command(mid=0x03, payload=bytes(payload))
        if not raw:
            return False

        # Phản hồi: MID=0x03, (M) U8 config result: 0=ok, 1=no support freq, 2=save failed
        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2
        result = raw[idx]
        return result == 0

    def query_rf_band(self) -> Optional[int]:
        """
        MID = 0x04: Query the RF frequency band of the Reader
        Response MID=0x04, (M) U8: current band code
        """
        raw = self._send_command(mid=0x04, payload=b"")
        if not raw:
            return None

        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2
        band = raw[idx]
        return band

    def set_working_frequency(self, auto_select: bool, freq_list: Optional[list] = None, save_persistence: bool = True) -> bool:
        """
        MID = 0x05: Configure the Working Frequency of the Reader
        Payload:
          - (M) U8: auto setting (0=off, 1=on)
          - Nếu auto=0, thêm PID=0x01 (Frequency List) U8[n]: danh sách channel (0..50 phần tử)
          - PID=0x02 (persistence) U8: 0=power down not save, 1=save
        """
        payload = bytearray()
        payload.append(1 if auto_select else 0)
        if not auto_select:
            # Cần freq_list (mỗi phần tử 1 byte)
            if not freq_list or not (1 <= len(freq_list) <= 50):
                raise ValueError("Frequency List must have 1..50 elements when auto_select=0")
            payload.append(0x01)  # PID frequency list
            payload.append(len(freq_list) & 0xFF)
            for ch in freq_list:
                payload.append(ch & 0xFF)
        # Add persistence
        payload.append(0x02)
        payload.append(1 if save_persistence else 0)

        raw = self._send_command(mid=0x05, payload=bytes(payload))
        if not raw:
            return False

        # Phản hồi: MID=0x05, (M) U8 config result: 0=ok, 1=chan not in band, 2=invalid freq, 3=other error, 4=save failed
        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2
        result = raw[idx]
        return result == 0

    def query_working_frequency(self) -> Dict[str, Any]:
        """
        MID = 0x06: Query the Working Frequency of the Reader
        Response MID=0x06, trả về:
          - (M) U8: auto setting (0/1)
          - (M) U8[n]: Frequency List (n phần tử) – chỉ khi auto=0
        """
        raw = self._send_command(mid=0x06, payload=b"")
        if not raw:
            return {}

        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2
        data = raw[idx: idx + data_len]
        cursor = 0
        auto_flag = data[cursor]
        cursor += 1
        freq_list = []
        if auto_flag == 0:
            # Toàn bộ còn lại là danh sách channel
            freq_list = list(data[cursor:])
        return {"auto": bool(auto_flag), "freq_list": freq_list}

    def set_tag_upload_params(self, filter_time: int, rssi_threshold: int) -> bool:
        """
        MID = 0x09: Configure the tag uploading parameter
        Payload:
          - PID=0x01 U16: repeated tag filtering time (0..65535, đơn vị 10ms)
          - PID=0x02 U8: RSSI threshold (0..255)
        """
        payload = bytearray()
        payload.append(0x01)
        payload += filter_time.to_bytes(2, byteorder='big')
        payload.append(0x02)
        payload.append(rssi_threshold & 0xFF)

        raw = self._send_command(mid=0x09, payload=bytes(payload))
        if not raw:
            return False

        # Phản hồi MID=0x09: cuối có (M) U8 config result: 0=ok, 1=param error, 2=save failed
        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2
        result = raw[idx]
        return result == 0

    def query_tag_upload_params(self) -> Dict[str, int]:
        """
        MID = 0x0A: Query the Tag Uploading Parameter
        Response MID=0x0A, trả về:
          - (M) U16: filter time
          - (M) U8: RSSI threshold
          - cuối (M) U8 config result: 0=ok, 1=param error, 2=save failed
        """
        raw = self._send_command(mid=0x0A, payload=b"")
        if not raw:
            return {}

        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2
        data = raw[idx: idx + data_len]
        filter_time = int.from_bytes(data[0:2], byteorder='big')
        rssi_thr = data[2]
        return {"filter_time": filter_time, "rssi_threshold": rssi_thr}

    def set_epc_baseband_param(self, speed: int, q_value: int, session: int, inventory_flag: int) -> bool:
        """
        MID = 0x0B: Configure EPC Baseband Parameter
        Payload:
          - PID=0x01 U8: EPC baseband speed (0..5 or 255)
          - PID=0x02 U8: default Q value (0..15)
          - PID=0x03 U8: Session (0..3)
          - PID=0x04 U8: inventory taking marking parameter (0..2)
        """
        payload = bytearray()
        payload.append(0x01)
        payload.append(speed & 0xFF)
        payload.append(0x02)
        payload.append(q_value & 0xFF)
        payload.append(0x03)
        payload.append(session & 0xFF)
        payload.append(0x04)
        payload.append(inventory_flag & 0xFF)

        raw = self._send_command(mid=0x0B, payload=bytes(payload))
        if not raw:
            return False

        # Phản hồi MID=0x0B, cuối có (M) U8 config result: 0=ok, 1=unsupported, 2=Q error, 3=session error, 4=inv flag error, 5=other, 6=save failed
        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2
        result = raw[idx]
        return result == 0

    def query_epc_baseband_param(self) -> Dict[str, int]:
        """
        MID = 0x0C: Query for EPC Baseband Parameter
        Response MID=0x0C, trả về:
          - (M) U8: EPC baseband speed
          - (M) U8: default Q value
          - (M) U8: Session
          - (M) U8: inventory taking marking parameter
          - (M) U8 config result: 0=ok, 1=unsupported, 2=Q error, 3=session error, 4=inv flag error, 5=other, 6=save failed
        """
        raw = self._send_command(mid=0x0C, payload=b"")
        if not raw:
            return {}

        idx = 1 + 4
        if self.RS485_FLAG:
            idx += 1
        data_len = int.from_bytes(raw[idx: idx + 2], byteorder='big')
        idx += 2
        data = raw[idx: idx + data_len]
        speed = data[0]
        q_val = data[1]
        session = data[2]
        inv_flag = data[3]
        return {"speed": speed, "q_value": q_val, "session": session, "inventory_flag": inv_flag}

    def read_epc_tag(self,
                     antennas: list,
                     single_read: bool = False,
                     match_params: Optional[bytes] = None,
                     tid_params: Optional[bytes] = None,
                     user_params: Optional[bytes] = None,
                     reserve_params: Optional[bytes] = None,
                     access_password: Optional[int] = None) -> None:
        """
        MID = 0x10: Read EPC Tag
        Payload cơ bản:
          - (M) U32: Bitmask các cổng antenna (ví dụ chỉ dùng antenna 1 → giá trị = 1<<0 = 0x01)
          - (M) U8: single/continuous (0=single, 1=continuous)
        Sau đó có thể lần lượt thêm:
          - PID=0x01, U8[n]: choose reading parameters (match area, address, bit length, dữ liệu)
          - PID=0x02, U8[2]: TID reading parameters
          - PID=0x03, U8[3]: user area reading parameters
          - PID=0x04, U8[3]: reserve area reading parameters
          - PID=0x05, U32: tag access password
          - PID=0x06..0x09 cho MONZA QT, RFMICRON, EM tags
        Khi reader đọc được tag, nó sẽ tự động upload EPC tag data (notification, MID=0x00)
        Khi kết thúc đọc, nó upload EPC reading end notification (MID=0x01).
        Ở SDK này, ta chỉ gửi command; việc xử lý notification/response thường thực hiện ở thread riêng.
        """
        # Tạo bitmask từ danh sách antennas
        mask = 0
        for ant in antennas:
            if 1 <= ant <= 32:
                mask |= (1 << (ant - 1))
        payload = bytearray()
        payload += mask.to_bytes(4, byteorder='big')
        payload.append(1 if single_read else 0)

        # Thêm các phần tử tùy chọn nếu có
        if match_params:
            payload.append(0x01)
            payload += match_params
        if tid_params:
            payload.append(0x02)
            payload += tid_params
        if user_params:
            payload.append(0x03)
            payload += user_params
        if reserve_params:
            payload.append(0x04)
            payload += reserve_params
        if access_password is not None:
            payload.append(0x05)
            payload += (access_password & 0xFFFFFFFF).to_bytes(4, byteorder='big')

        # Gửi lệnh
        self._send_command(mid=0x10, payload=bytes(payload))

    def stop_read(self) -> None:
        """
        MID = 0xFF: Stop Command – dừng mọi hoạt động RFID, reader về idle.
        """
        self._send_command(mid=0xFF, payload=b"")


if __name__ == "__main__":
    # Ví dụ sử dụng
    reader = NationReader(port="/dev/ttyUSB0", baudrate=115200, timeout=0.3)
    try:
        reader.connect()

        # 1. Gửi Stop để chắc reader về Idle
        reader.stop_read()

        # 2. Query Reader Info
        info = reader.query_reader_info()
        print("Reader Info:", info)

        # 3. Query Baseband Version
        bbv = reader.query_baseband_version()
        print("Baseband Version:", hex(bbv) if bbv is not None else None)

        # 4. Cấu hình công suất cho antenna 1 = 20 dBm, antenna 2 = 18 dBm
        ok = reader.set_reader_power({1: 20, 2: 18})
        print("Set power:", "OK" if ok else "FAILED")

        # 5. Query Power
        pw = reader.query_reader_power()
        print("Current Powers:", pw)

        # 6. Cấu hình RF band = FCC (code=3)
        ok = reader.set_rf_band(3)
        print("Set RF Band:", "OK" if ok else "FAILED")

        # 7. Query RF band
        band = reader.query_rf_band()
        print("Current RF Band:", band)

        # 8. Cấu hình working frequency auto=0, list=[0,7,15] (ví dụ)
        ok = reader.set_working_frequency(auto_select=False, freq_list=[0, 7, 15])
        print("Set Working Frequency:", "OK" if ok else "FAILED")

        # 9. Query working frequency
        wf = reader.query_working_frequency()
        print("Working Frequency:", wf)

        # 10. Thiết lập tag uploading params: filter_time=100 (1s), rssi_threshold=10
        ok = reader.set_tag_upload_params(filter_time=100, rssi_threshold=10)
        print("Set Tag Upload Params:", "OK" if ok else "FAILED")

        # 11. Query tag uploading params
        tup = reader.query_tag_upload_params()
        print("Tag Upload Params:", tup)

        # 12. Cấu hình EPC baseband: speed=1, q=4, session=0, inv_flag=2
        ok = reader.set_epc_baseband_param(speed=1, q_value=4, session=0, inventory_flag=2)
        print("Set EPC Baseband Param:", "OK" if ok else "FAILED")

        # 13. Query EPC baseband param
        ebp = reader.query_epc_baseband_param()
        print("EPC Baseband Param:", ebp)

        # 14. Bắt đầu đọc EPC tags trên antenna 1, single read
        reader.read_epc_tag(antennas=[1], single_read=True)

        # 15. Dừng đọc
        reader.stop_read()

    except Exception as e:
        print("[ERROR]", e)
    finally:
        reader.close()
