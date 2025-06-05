from nation_sdkv2 import RFIDReader

# 1. Instantiate (port, baudrate, RS485 address, optional timeout)
reader = RFIDReader(port="/dev/ttyUSB0", baudrate=115200, address=0x01, timeout=1.0)

try:
    # 2. Open serial connection
    reader.open()

    # 3. Query basic info
    info = reader.query_reader_info()
    print("Raw Reader Info:", info['raw_params'].hex())

    # 4. Set transmit power to 20 dBm
    ok = reader.set_reader_power(20)
    print("Set power OK?" , ok)

    # # 5. Enable antenna #1
    # ok = reader.set_antenna(0x01)
    # print("Set antenna OK?", ok)

    # # 6. Perform a quick EPC inventory
    # tags = reader.read_epc_tag(antenna_mask=0x01, read_duration_ms=500)
    # print("Tags found:")
    # for t in tags:
    #     print(f"  EPC={t['epc']}, RSSI={t['rssi']}")

    # # 7. Stop inventory
    # reader.stop()

    # 8. Reboot reader if needed
    # rebooted = reader.reboot_reader()
    # print("Reboot OK?", rebooted)

finally:
    # 9. Close serial
    reader.close()
