#!/usr/bin/env python3
# reader_sdk.py

"""
Python SDK for “RFID Reader Data Communication Protocol” (all reader models).

Implements:
 - Frame structure: header (0x5A), control word (4 bytes), [address], length (2 bytes), payload, CRC16 (2 bytes)
 - U8, S8, U16, S16, U32, S32 packing/unpacking in big-endian
 - CCITT-16 CRC (poly=0x1021, init=0x0000, MSB-first)
 - Generic send/receive over serial (pyserial)
 - Example high-level commands: query_reader_info, set_reader_power, read_epc_tag, stop, etc.

Usage:
    from reader_sdk import RFIDReader
    reader = RFIDReader(port="/dev/ttyUSB0", baudrate=115200, address=0x01)
    reader.open()
    info = reader.query_reader_info()
    print("Reader Info:", info)
    reader.set_reader_power(power_level=20)      # e.g. 20 dBm
    tags = reader.read_epc_tag(antenna_mask=0x01)  # read on antenna 1
    print("Tags:", tags)
    reader.stop()
    reader.close()
"""

import struct
import serial
import threading
import time

# =============================================================================
# CRC16-CCITT (X^16 + X^15 + X^2 + 1), init=0x0000, MSB-first
# =============================================================================
def crc16_ccitt(data: bytes, poly: int = 0x1021, init_val: int = 0x0000) -> int:
    """
    Compute CRC16-CCITT (MSB-first) over `data`, with given polynomial and initial value.
    """
    crc = init_val
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if (crc & 0x8000) != 0:
                crc = ((crc << 1) & 0xFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF

# =============================================================================
# Helper functions for packing/unpacking different types in big-endian
# =============================================================================
def pack_u8(value: int) -> bytes:
    return struct.pack('>B', value & 0xFF)

def pack_s8(value: int) -> bytes:
    return struct.pack('>b', value & 0xFF)

def pack_u16(value: int) -> bytes:
    return struct.pack('>H', value & 0xFFFF)

def pack_s16(value: int) -> bytes:
    return struct.pack('>h', value & 0xFFFF)

def pack_u32(value: int) -> bytes:
    return struct.pack('>I', value & 0xFFFFFFFF)

def pack_s32(value: int) -> bytes:
    return struct.pack('>i', value & 0xFFFFFFFF)

def unpack_u8(buffer: bytes, offset: int = 0) -> (int, int):
    return struct.unpack_from('>B', buffer, offset)[0], offset + 1

def unpack_u16(buffer: bytes, offset: int = 0) -> (int, int):
    return struct.unpack_from('>H', buffer, offset)[0], offset + 2

def unpack_u32(buffer: bytes, offset: int = 0) -> (int, int):
    return struct.unpack_from('>I', buffer, offset)[0], offset + 4

# =============================================================================
# Main class: RFIDReader
# =============================================================================
class RFIDReader:
    FRAME_HEADER = 0x5A

    # Protocol Type Number (2 bits): 00 for UHF RFID reader protocol
    PROTOCOL_TYPE = 0b00 << 30

    # Protocol Version No. (6 bits) — typically 0x01
    PROTOCOL_VERSION = 0x01 << 24  # shift into bits 29..24

    # Bits within the 4-byte Protocol Control Word:
    #   RS485 Flag Bit (bit 23)
    #   Reader notification Flag Bit (bit 22)
    #   Message Category Number (5 bits: bits 21..17)
    #   Message ID (MID) (8 bits: bits 7..0)
    #
    # We'll construct the 4-byte control word as:
    #   bits 31..30 = protocol type
    #   bits 29..24 = version
    #   bit 23      = RS485 flag (1 if using RS485 addressing)
    #   bit 22      = notification flag (usually 0)
    #   bits21..17  = category
    #   bits16..8   = reserved (0)
    #   bits7..0    = MID
    #
    # For simplicity, we mask/reserve bits16..8 as zero.

    def __init__(self, port: str, baudrate: int = 115200, address: int = 0x01, timeout: float = 0.5):
        """
        :param port: Serial port (e.g. "/dev/ttyUSB0" or "COM3")
        :param baudrate: Baud rate for serial communication
        :param address: 1-byte device address (0x00..0xFF). Only used if RS485 flag = 1.
        :param timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.address = address & 0xFF
        self.timeout = timeout

        self._ser = None
        self._lock = threading.Lock()  # ensure thread-safe send/receive if needed

    def open(self):
        """Open the serial port."""
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout
        )
        # Flush any lingering data
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    def close(self):
        """Close the serial port."""
        if self._ser and self._ser.is_open:
            self._ser.close()
            self._ser = None

    def _compute_control_word(self, rs485_flag: int, notify_flag: int,
                              category: int, mid: int) -> bytes:
        """
        Build the 4-byte Protocol Control Word.
        :param rs485_flag: 0 or 1
        :param notify_flag: 0 or 1
        :param category: 5-bit Message Category Number (0..31)
        :param mid: 8-bit Message ID (0..255)
        :return: 4 bytes, big-endian
        """
        # Sanity checks
        rs485_flag &= 0x01
        notify_flag &= 0x01
        category &= 0x1F
        mid &= 0xFF

        # Bits 31..30: protocol type (two bits)
        word = (self.PROTOCOL_TYPE & 0xC0000000)

        # Bits 29..24: version (six bits)
        word |= (self.PROTOCOL_VERSION & 0x3F000000)

        # Bit 23: RS485 flag
        word |= (rs485_flag << 23)

        # Bit 22: notification flag
        word |= (notify_flag << 22)

        # Bits 21..17: category
        word |= (category & 0x1F) << 17

        # Bits 16..8: reserved (0)
        # Bits 7..0: MID
        word |= mid

        # Pack as big-endian unsigned 32-bit
        return struct.pack('>I', word)

    def _build_frame(self, mid: int, category: int, data_params: bytes = b'',
                     rs485_flag: int = 0, notify_flag: int = 0) -> bytes:
        """
        Construct a full frame to send:
        [0x5A][4-byte control word][(address if RS485=1)][2-byte length][payload][2-byte CRC]
        :param mid: 8-bit Message ID
        :param category: 5-bit Message Category Number
        :param data_params: raw payload bytes (parameters)
        :param rs485_flag: 0 or 1 (include address)
        :param notify_flag: 0 or 1
        :return: full frame bytes
        """
        header = struct.pack('>B', self.FRAME_HEADER)
        ctrl = self._compute_control_word(rs485_flag, notify_flag, category, mid)

        # If RS485 flag set, include 1-byte address
        addr_bytes = b''
        if rs485_flag:
            addr_bytes = struct.pack('>B', self.address)

        # Length = len(data_params) + (addr included? 0 : 0)
        # Note: “data length” field is only for the data parameters, not including header, control word, address, or CRC.
        length_field = pack_u16(len(data_params))

        frame_wo_crc = header + ctrl + addr_bytes + length_field + data_params
        # Compute CRC over everything after the header (i.e., ctrl + [addr] + length + data_params).
        # The protocol says: Check Code (2 bytes, CRC-16 CCITT) computed over Frame Header EXCLUDED. So: bytes from ctrl through last data byte.
        crc_input = frame_wo_crc[1:]
        checksum = crc16_ccitt(crc_input)
        crc_bytes = pack_u16(checksum)

        return frame_wo_crc + crc_bytes

    def _parse_frame(self, raw: bytes) -> dict:
        """
        Parse a received frame. Raises ValueError if invalid.
        Returns a dict with fields:
          - category, mid, rs485_flag, notify_flag, address (if present), params (bytes)
        """
        if len(raw) < 1 + 4 + 2 + 2:
            raise ValueError("Frame too short to be valid")

        # 1-byte header
        header = raw[0]
        if header != self.FRAME_HEADER:
            raise ValueError(f"Invalid frame header: 0x{header:02X}")

        # 4-byte control word
        ctrl_word = raw[1:5]
        word = struct.unpack('>I', ctrl_word)[0]

        # Extract fields
        protocol_type = (word >> 30) & 0x03
        version = (word >> 24) & 0x3F
        rs485_flag = (word >> 23) & 0x01
        notify_flag = (word >> 22) & 0x01
        category = (word >> 17) & 0x1F
        mid = word & 0xFF

        idx = 5
        address = None
        if rs485_flag:
            if len(raw) < idx + 1:
                raise ValueError("Frame missing address byte")
            address = raw[idx]
            idx += 1

        # Next 2 bytes: length of data parameters
        if len(raw) < idx + 2:
            raise ValueError("Frame missing length field")
        data_len, = struct.unpack('>H', raw[idx:idx+2])
        idx += 2

        # Ensure full length is present: idx + data_len + 2 (for CRC)
        if len(raw) < idx + data_len + 2:
            raise ValueError("Incomplete frame (data + CRC missing)")

        params = raw[idx: idx + data_len]
        idx += data_len

        # CRC from received
        recv_crc, = struct.unpack('>H', raw[idx:idx+2])
        # Compute CRC over everything except header (i.e., bytes[1: idx])
        calc_crc = crc16_ccitt(raw[1: idx])
        if recv_crc != calc_crc:
            raise ValueError(f"CRC mismatch: received 0x{recv_crc:04X}, calculated 0x{calc_crc:04X}")

        return {
            'protocol_type': protocol_type,
            'version': version,
            'rs485_flag': rs485_flag,
            'notify_flag': notify_flag,
            'category': category,
            'mid': mid,
            'address': address,
            'params': params
        }

    def _send_raw(self, frame: bytes):
        """
        Write the full frame to serial.
        """
        if not self._ser or not self._ser.is_open:
            raise IOError("Serial port not open")
        with self._lock:
            self._ser.write(frame)

    def _receive_raw(self) -> bytes:
        """
        Read from serial until a full frame is captured (or timeout). Returns raw bytes.
        Assumes reader sends: [0x5A][...][CRC(2 bytes)].
        This implementation:
          - Read at least 1 byte; when 0x5A is found, peek at next 4 + ... bytes to get length, etc.
          - Because serial reads are streaming, we accumulate into a buffer until we can parse one frame.
        """
        if not self._ser or not self._ser.is_open:
            raise IOError("Serial port not open")

        buf = bytearray()
        start_time = time.time()
        while True:
            # Attempt to read one byte
            byte = self._ser.read(1)
            if byte:
                buf += byte
                # If we have at least header + control (1+4) + maybe address + length (2), try to parse length
                if len(buf) >= 1 + 4 + 2:
                    # Check header
                    if buf[0] != self.FRAME_HEADER:
                        # Discard until we find next 0x5A
                        idx = buf.find(bytes([self.FRAME_HEADER]), 1)
                        if idx == -1:
                            buf = bytearray()
                        else:
                            buf = buf[idx:]
                        continue

                    # Extract control word to see if RS485 flag is set
                    if len(buf) < 1 + 4:
                        continue  # need more bytes
                    ctrl_word = buf[1:5]
                    word = struct.unpack('>I', ctrl_word)[0]
                    rs485_flag = (word >> 23) & 0x01
                    # Calculate where length bytes should be
                    expected_len_idx = 1 + 4 + (1 if rs485_flag else 0)
                    if len(buf) < expected_len_idx + 2:
                        continue
                    data_len = struct.unpack('>H', buf[expected_len_idx: expected_len_idx + 2])[0]
                    # Total frame size = header (1) + ctrl (4) + [addr?1] + length (2) + data_len + CRC (2)
                    total_len = 1 + 4 + (1 if rs485_flag else 0) + 2 + data_len + 2
                    if len(buf) < total_len:
                        continue  # wait until full frame arrives
                    # We have at least one full frame
                    frame = bytes(buf[:total_len])
                    # Remove this frame from buffer for next reads
                    # (IMPORTANT if multiple frames come back to back)
                    del buf[:total_len]
                    return frame
            else:
                # Timeout condition
                if (time.time() - start_time) > self.timeout:
                    raise TimeoutError("Timeout waiting for response")

    def _send_command(self, mid: int, category: int, params: bytes = b'',
                    rs485_flag: int = 0, notify_flag: int = 0) -> dict:
        frame = self._build_frame(mid=mid, category=category, data_params=params,
                                rs485_flag=rs485_flag, notify_flag=notify_flag)
        print(f"[SEND] Frame sent: {frame.hex()}")  # Debugging: Print the frame being sent
        self._send_raw(frame)

        raw_resp = self._receive_raw()
        print(f"[RECV] Frame received: {raw_resp.hex()}")  # Debugging: Print the response frame
        parsed = self._parse_frame(raw_resp)
        return parsed
    # =============================================================================
    # High-level command implementations
    # =============================================================================

    # ---- Category 0x00: Reader Configuration Management ----

    def query_reader_info(self) -> dict:
        """
        MID 0x00, Category=0x00: Query Reader Information
        Response params (for example) might include:
          - Reader Serial Number (string, fixed length or length+content)
          - Reader Model, Firmware Version, etc.
        Here we just return raw params; parsing them depends on spec details (not fully enumerated).
        """
        resp = self.send_command(mid=0x00, category=0x00, params=b'')
        # The 'params' field is raw; you need to parse per protocol spec:
        # e.g., first U16 length of SN, SN bytes, U8 model code, U8 fw_major, U8 fw_minor, etc.
        return {'raw_params': resp['params']}

    def set_serial_port_params(self, baud_rate: int, data_bits: int, parity: int, stop_bits: int) -> bool:
        """
        MID 0x02, Category=0x00: Configure serial port parameters
        Params (example):
          U32 baud_rate, U8 data_bits, U8 parity, U8 stop_bits
        """
        payload = pack_u32(baud_rate) + pack_u8(data_bits) + pack_u8(parity) + pack_u8(stop_bits)
        resp = self.send_command(mid=0x02, category=0x00, params=payload)
        # Check status code: often first byte of resp['params'] is status (U8: 0x00=OK)
        status_code = resp['params'][0]
        return (status_code == 0x00)

    def reboot_reader(self) -> bool:
        """
        MID 0x0F, Category=0x00: Restart the reader
        No params; response U8 status code.
        """
        resp = self.send_command(mid=0x0F, category=0x00, params=b'')
        status = resp['params'][0]
        return (status == 0x00)

    # ---- Category 0x01: RFID Configuration and Operation ----

    def query_rfid_ability(self) -> dict:
        """
        MID 0x00, Category=0x01: Query Reader's RFID Ability
        Returns raw params. Parse per spec if needed.
        """
        resp = self.send_command(mid=0x00, category=0x01, params=b'')
        return {'raw_params': resp['params']}

    def set_reader_power(self, power_level: int) -> bool:
        """
        MID 0x01, Category=0x01: Configure the Reader Power
        Params: U8 power (0..max)
        """
        payload = pack_u8(power_level)
        resp = self.send_command(mid=0x01, category=0x01, params=payload)
        status = resp['params'][0]
        return (status == 0x00)

    def set_rf_band(self, rf_band: int) -> bool:
        """
        MID 0x03, Category=0x01: Configure the RF Band
        Params: U8 rf_band (0: FCC, 1: ETSI, etc., per spec)
        """
        payload = pack_u8(rf_band)
        resp = self.send_command(mid=0x03, category=0x01, params=payload)
        status = resp['params'][0]
        return (status == 0x00)

    def set_antenna(self, antenna_mask: int) -> bool:
        """
        MID 0x07, Category=0x01: Configure the antenna
        Params: U8 antenna_mask (bitmask of antennas to enable)
        """
        payload = pack_u8(antenna_mask)
        resp = self.send_command(mid=0x07, category=0x01, params=payload)
        status = resp['params'][0]
        return (status == 0x00)

    def read_epc_tag(self, antenna_mask: int, read_duration_ms: int = 500) -> list:
        """
        MID 0x10, Category=0x01: Read EPC Tag
        Params:
          U8 antenna_mask
          U16 read_duration (ms)
        Returns: list of EPCs read (parsed as hex strings)
        Note: actual param format may include additional fields—adjust as per spec.
        """
        payload = pack_u8(antenna_mask) + pack_u16(read_duration_ms)
        resp = self.send_command(mid=0x10, category=0x01, params=payload)

        data = resp['params']
        offset = 0
        tags = []

        # Example parsing: first U16 = number of tags
        if len(data) < 2:
            return tags

        num_tags, offset = unpack_u16(data, offset)
        for _ in range(num_tags):
            # Each tag: U16 epc_len, EPC bytes (epc_len), U8 RSSI
            if offset + 2 > len(data):
                break
            epc_len, offset = unpack_u16(data, offset)
            if offset + epc_len > len(data):
                break
            epc_bytes = data[offset: offset + epc_len]
            offset += epc_len
            epc_hex = epc_bytes.hex().upper()
            # Optionally: parse RSSI
            if offset + 1 <= len(data):
                rssi = data[offset]
                offset += 1
            else:
                rssi = None
            tags.append({'epc': epc_hex, 'rssi': rssi})

        return tags

    def write_epc_tag(self, antenna: int, epc_hex: str) -> bool:
        """
        MID 0x11, Category=0x01: Write EPC Tag
        Params:
          U8 antenna
          U16 epc_length
          N bytes EPC
        """
        epc_bytes = bytes.fromhex(epc_hex)
        payload = pack_u8(antenna) + pack_u16(len(epc_bytes)) + epc_bytes
        resp = self.send_command(mid=0x11, category=0x01, params=payload)
        status = resp['params'][0]
        return (status == 0x00)

    def lock_epc_tag(self, antenna: int, lock_type: int) -> bool:
        """
        MID 0x12, Category=0x01: Lock EPC Tag
        Params:
          U8 antenna, U8 lock_type
        """
        payload = pack_u8(antenna) + pack_u8(lock_type)
        resp = self.send_command(mid=0x12, category=0x01, params=payload)
        status = resp['params'][0]
        return (status == 0x00)

    def kill_epc_tag(self, antenna: int, kill_password: int) -> bool:
        """
        MID 0x13, Category=0x01: Kill EPC Tag
        Params:
          U8 antenna, U32 kill_password
        """
        payload = pack_u8(antenna) + pack_u32(kill_password)
        resp = self.send_command(mid=0x13, category=0x01, params=payload)
        status = resp['params'][0]
        return (status == 0x00)

    def stop(self) -> bool:
        """
        MID 0xFF, Category=0x01: Stop Command (stop any current inventory/read)
        No params; response U8 status
        """
        resp = self.send_command(mid=0xFF, category=0x01, params=b'')
        status = resp['params'][0]
        return (status == 0x00)

    # ---- Category 0x02: Upgrade (examples) ----

    def upgrade_reader_app(self, data_chunk: bytes, sequence: int) -> bool:
        """
        MID 0x00, Category=0x02: Reader application software upgrading
        Payload format depends on protocol (e.g., U16 chunk_length + chunk_data + U16 sequence).
        Adjust per spec.
        """
        chunk_len = len(data_chunk)
        payload = pack_u16(chunk_len) + data_chunk + pack_u16(sequence)
        resp = self.send_command(mid=0x00, category=0x02, params=payload)
        status = resp['params'][0]
        return (status == 0x00)

    # ---- Category 0x03: Testing Commands (examples) ----

    def start_carrier(self, antenna: int) -> bool:
        """
        MID 0x00, Category=0x03: Transmit carrier instructions
        Params: U8 antenna
        """
        payload = pack_u8(antenna)
        resp = self.send_command(mid=0x00, category=0x03, params=payload)
        status = resp['params'][0]
        return (status == 0x00)

    def stop_carrier(self) -> bool:
        """
        MID 0x01, Category=0x03: Stop carrier
        No params
        """
        resp = self.send_command(mid=0x01, category=0x03, params=b'')
        status = resp['params'][0]
        return (status == 0x00)

    # ... Add other categories/commands in exactly the same pattern ...

# In reader_sdk.py, inside class RFIDReader:

    # ---- Category 0x01: RFID Configuration and Operation ----

    def get_reader_power(self) -> dict:
        """
        Query the Reader Power (MID=0x02, Category=0x01).
        No payload in request. The reader responds with one U8 power value per antenna port,
        tagged by PID = {0x01, 0x02, ..., 0x40}. Each value is 0..36 (dBm).
        Returns: { port_index (1..64): power_dBm, ... }
        """
        # 1. Send command with MID=0x02, Category=0x01, no params
        resp = self.send_command(mid=0x02, category=0x01, params=b'')
        data = resp['params']
        offset = 0
        powers = {}

        # Parse until end of data. Each param is 2 bytes: [PID (U8)] + [power (U8)].
        # Stop if data length is not a multiple of 2.
        if len(data) % 2 != 0:
            raise ValueError("Invalid payload length in get_reader_power response")

        while offset < len(data):
            pid = data[offset]          # U8: PID (1..64)
            power = data[offset + 1]    # U8: power in dBm
            offset += 2

            # Only accept PIDs in 1..64
            if 1 <= pid <= 64:
                powers[pid] = power
            else:
                # Unknown PID—ignore or log if needed
                pass

        return powers

    def set_reader_power(self,
                         ports: dict,
                         persist: bool = True) -> bool:
        """
        Configure the Reader Power (MID=0x01, Category=0x01).
        :param ports: dict mapping port_index (1..64) -> desired_power (0..36)
                      e.g. {1: 20, 2: 18, 3: 30}
        :param persist: if True, include PID=0xFF + U8(1) so reader saves after power‐down.
                        if False, include PID=0xFF + U8(0) so it does not save.
        :returns: True if configuration successful (status=0), False otherwise.
        """
        payload = bytearray()

        # 1. For each port in [1..64], append: [PID=port_index (U8)] + [power (U8)]
        for port_idx, pwr in ports.items():
            if not (1 <= port_idx <= 64):
                raise ValueError(f"Invalid port index {port_idx}; must be 1..64")
            if not (0 <= pwr <= 36):
                raise ValueError(f"Invalid power {pwr}; must be 0..36 dBm")

            payload += pack_u8(port_idx)  # PID = U8(1..64)
            payload += pack_u8(pwr)       # U8 power

        # 2. Append the “persistence” parameter (PID=0xFF, U8 length=1) if desired.
        #    PID=0xFF is reserved for “save after power‐down”:
        #      0xFF + U8(0) => do NOT save after power‐down
        #      0xFF + U8(1) => save after power‐down
        if persist:
            payload += pack_u8(0xFF)  # PID for “save flag”
            payload += pack_u8(1)     # U8 = 1 => save
        else:
            payload += pack_u8(0xFF)
            payload += pack_u8(0)     # U8 = 0 => do not save

        # 3. Send command
        resp = self.send_command(mid=0x01, category=0x01, params=bytes(payload))

        # 4. The reader’s response payload should be a single U8 “status code”:
        #    0 = success
        #    1 = hardware does not support the specified port parameter
        #    2 = reader does not support the specified power parameter
        #    3 = save parameter failed
        if len(resp['params']) < 1:
            raise ValueError("No status byte in set_reader_power response")

        status = resp['params'][0]
        return (status == 0x00)

# =============================================================================
# Example usage (remove or comment out in production)
# =============================================================================
if __name__ == "__main__":
    # Example: change "/dev/ttyUSB0" to your actual port, and address to whichever the reader uses (0x01 by default).
    reader = RFIDReader(port="/dev/ttyUSB0", baudrate=115200, address=0x01, timeout=3.0)
    try:
        reader.open()
        print("Querying reader info...")
        info = reader.query_reader_info()
        print("Raw Reader Info Params:", info['raw_params'].hex())

        print("Setting reader power to 20 dBm...")
        if reader.set_reader_power(20):
            print("Power set successfully.")
        else:
            print("Failed to set power.")

        print("Starting inventory (read EPC tags) on antenna 1...")
        tags = reader.read_epc_tag(antenna_mask=0x01, read_duration_ms=500)
        print(f"Tags read ({len(tags)}):")
        for t in tags:
            print(f"  EPC={t['epc']}, RSSI={t['rssi']}")

        print("Stopping inventory...")
        reader.stop()

    except Exception as e:
        print("Error:", e)
    finally:
        reader.close()
        print("Serial port closed.")
