import serial 
import time
import os
COM_PORT = "/dev/ttyUSB1"
BAUD_RATE = 9600
#FILE_PATH = r"C:\Temp\Label_output.ezpl"
label_data = """
^AT
^O0
^D0
^C1
^P1
^Q16.0,10.0
^W24
^L
RFW,H,2,24,1,00000001749021006060A1
W64,45,5,2,L,3,3,36,0
thuocsi.vn/qr/00000001749021006060A1
E
"""
# label_data = """
# ^AT
# ^O0
# ^D0
# ^C1
# ^P1
# ^Q16.0,10.0
# ^W24
# ^L
# RFW,H,2,24,1,00000001749021006060A1
# W64,45,5,2,L,8,3,36,0
# thuocsi.vn/qr/00000001749021006060A1
# E
# """

def send_ezpl_file():
    try:
        with serial.Serial(COM_PORT, BAUD_RATE, timeout=1) as ser:
            ser.write(label_data.encode("ascii"))
            ser.flush()
            print(f"da gui file qua {COM_PORT}")
    except Exception as e:
            print(f"Error send file: {e}")

if __name__ == "__main__":
    send_ezpl_file()
