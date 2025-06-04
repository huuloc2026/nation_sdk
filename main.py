from nation_sdk import NationReader

def main():
    port = "/dev/ttyUSB0"
    baudrate = 9600
    reader = NationReader(port,baudrate=baudrate)
    
    try:
        reader.connect()

        # # # 1️⃣ STOP để chuyển về IDLE
        reader.stop()

        # # # 2️⃣ Query thông tin thiết bị
        reader.query_info()

        # # # 3️⃣ Query phiên bản baseband
        reader.query_baseband_version()

        # # # 4️⃣ Query baudrate hiện tại
        reader.query_baudrate()

        # # # 5️⃣ Đặt baudrate nếu cần (2 = 115200)
        reader.set_baudrate(0)

        # # # 6️⃣ Cấu hình công suất antenna
        reader.set_power({
            1: 30,

        }, persistent=True)


        
        # # 7️⃣ Bắt đầu inventory
        if reader.start_inventory():
            print("🛰️ Inventory running... Đang chờ thẻ RFID")

            while True:
                response = reader.ser.read(128)
                if not response or len(response) < 8:
                    continue

                mid = response[6]
                if mid == 0x10:  # Inventory data
                    payload = response[7:-2]
                    tag_list = reader.parse_inventory_data(list(payload))
                    for tag in tag_list:
                        print(f"🎯 EPC: {tag['epc']}  RSSI: {tag['rssi']} dBm")
                        
    except KeyboardInterrupt:
        print("⏹️ Inventory stopped by user.")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        reader.close()

if __name__ == "__main__":
    main()
