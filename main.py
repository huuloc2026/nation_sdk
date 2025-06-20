from nation_sdk import NationReader
import time
import json
import threading
import subprocess

def beep():
    try:
        subprocess.Popen(
            ["ffplay", "-nodisp", "-autoexit", "beep.mp3"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
    except Exception as e:
        print(f"⚠️ Beep error: {e}")
        
def on_tag_callback(tag: dict) -> str:
    payload = {
        "epc": tag.get("epc"),
        "rssi": tag.get("rssi"),
        "antenna_id": tag.get("antenna_id"),
        "status": "tag_detected"
    }
    print(json.dumps(payload))
    try:
        if reader.get_beeper(1):
            threading.Thread(target=beep, daemon=True).start()
    except Exception as e:
        print(f"⚠️ Buzzer error: {e}")

    return json.dumps(payload)

def on_end_callback(reason):
    reasons = {
        0: "Kết thúc do đọc 1 lần",
        1: "Dừng bởi lệnh STOP",
        2: "Lỗi phần cứng"
    }
    print(f"📴 Inventory kết thúc. Lý do: {reasons.get(reason, 'Không rõ')}")
    
    
def main():
    global reader
    port = "/dev/ttyUSB0"
    baud = 115200
    reader = NationReader(port, baud)
    

    reader.open()
    print("🔧 Connecting and initializing reader...")
    if not reader.Connect_Reader_And_Initialize():
        print("❌ Initialization failed.")
        return


    # Enable các anten hub 1, 2, 3
    # for a in [1, 2, 3]:
    #     reader.enable_ant(a)

    
    
    
    # print("✅ Danh sách anten đang bật:", reader.get_enabled_ants())
    
    # info = reader.Query_Reader_Information()
    # print("📡 Reader Info:")
    # for k, v in info.items():
    #     print(f"  {k}: {v}")


    setPower = {
        1:33, 
        2:1,
        3:1,
        4:1
    }
    reader.configure_reader_power(setPower, persistence=True)
    # powers = reader.query_reader_power()
    # for ant in range(1, 5):
    #     val = powers.get(ant)
    #     if val is not None:
    #         print(f"  🔧 Antenna {ant}: {val} dBm")
    #     else:
    #         print(f"  ⚠️ Antenna {ant}: N/A")
    
    reader.stop_inventory()
    
    
    
    
    try:
        print("▶️ Bắt đầu đọc tag (ấn Ctrl+C để dừng)...")
        reader.start_inventory(on_tag=on_tag_callback, on_inventory_end=on_end_callback)
        time.sleep(1000)
        
    except KeyboardInterrupt:
        reader.stop_inventory()
        print("\n🛑 Đã nhận Ctrl+C – dừng đọc tag...")
        
    finally:
        success = reader.stop_inventory()
        
        if success:
            print("✅ Inventory đã dừng thành công")
        else:
            print("❌ Không thể dừng reader")
        reader.close()
        print("🔌 Đóng kết nối UART")


if __name__ == "__main__":
    main()
