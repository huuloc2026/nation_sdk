import serial
import struct

def crc16_ccitt_msb(data: bytes, poly=0x1021, init=0x0000) -> int:
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

# RS485 MID=0x00, addr=0x00
frame = b'\x5A'                         # Header
frame += b'\x00\x01\x00\x00'           # Protocol Control Word (RS485, MID=0x00)
frame += b'\x00'                        # Address
frame += b'\x00\x00'                    # Length (0 byte payload)
crc = crc16_ccitt_msb(frame[1:])
frame += crc.to_bytes(2, 'big')        # CRC16 MSB

print("[SEND]", frame.hex())

ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=0.5)
ser.write(frame)
response = ser.read(128)
print("[RECV]", response.hex())
