from nation_sdk import NationReader
import time

def split_frames(raw: bytes) -> list[bytes]:
    """
    Splits concatenated frames from the RFID reader response stream.
    Each frame starts with 0x5A and ends after known data length + CRC.
    """
    frames = []
    i = 0
    while i < len(raw):
        if raw[i] != 0x5A:
            i += 1
            continue

        # Minimum frame size
        if i + 9 > len(raw):
            break

        # Extract PCW
        pcw = int.from_bytes(raw[i+1:i+5], 'big')
        rs485_flag = (pcw >> 13) & 0x01
        len_offset = i + 5 + (1 if rs485_flag else 0)
        if len(raw) < len_offset + 2:
            break  # Incomplete

        data_len = int.from_bytes(raw[len_offset:len_offset+2], 'big')
        frame_len = 1 + 4 + (1 if rs485_flag else 0) + 2 + data_len + 2  # Header + PCW + Addr? + Len + Data + CRC

        if i + frame_len > len(raw):
            break  # Frame incomplete

        frames.append(raw[i:i + frame_len])
        i += frame_len
    return frames

def debug_crc(frame: bytes):
    """
    Print calculated vs actual CRC for debugging
    """
    if len(frame) < 9: return
    body = frame[1:-2]
    actual_crc = int.from_bytes(frame[-2:], 'big')
    expected_crc = NationReader.crc16_ccitt(body)
    print(f"🔬 Frame CRC: actual=0x{actual_crc:04X}, expected=0x{expected_crc:04X}")

def main():
    port = "/dev/ttyUSB0"
    baudrate = 9600

    print("🔧 Attempting to connect and initialize Nation RFID reader...\n")
    


    success = NationReader.Connect_Reader_And_Initialize(port=port, baudrate=baudrate)

    if success:
        print("🎉 Test PASSED: Reader is initialized and ready.")
    else:
        print("❌ Test FAILED: Reader did not respond correctly.")


if __name__ == "__main__":
    main()
