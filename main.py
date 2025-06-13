from nation_sdk import NationReader
import time

def main():
    reader = NationReader("/dev/ttyUSB0", 9600)
    reader.open()

    print("🔧 Connecting and initializing reader...")
    if reader.Connect_Reader_And_Initialize():
        # info = reader.Query_Reader_Information()
        # print("📡 Reader Info:")
        # for k, v in info.items():
        #     print(f"  {k}: {v}")

        print("\n🚀 Starting inventory for 10 seconds...")
        reader.start_inventory()

        # Let inventory run
        time.sleep(100)

        print("🛑 Stopping inventory...")
        reader.stop_inventory()
        print("✅ Inventory session complete.")

    else:
        print("❌ Initialization failed.")

    reader.close()
    print("🔌 Reader connection closed.")

if __name__ == "__main__":
    main()
