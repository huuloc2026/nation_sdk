from nation_sdk import NationReader
import time

def main():

    port = "/dev/ttyUSB0"
    braud = 9600
    reader = NationReader(port, braud)
    reader.open()

    print("🔧 Connecting and initializing reader...")
    if reader.Connect_Reader_And_Initialize():
        # info = reader.Query_Reader_Information()
        # print("📡 Reader Info:")
        # for k, v in info.items():
        #     print(f"  {k}: {v}")


    


        #  # === (1) Query Power Range ===
        print("📡 power_range Info:")
        power_range = reader.query_power_range()
        print(power_range)
        # if power_range:
        #     min_pwr, max_pwr = power_range
        #     print(f"✅ Supported power range: {min_pwr}–{max_pwr} dBm")
        # else:
        #     print("⚠️ Could not determine power range.")
        

        # reader.set_power(ant=1, power=30)
        # # === (2) Get All Powers ===
        # print("\n🔍 Reading current power settings for all ports...")
        # powers = reader.get_all_powers()
        # for ant, pwr in powers.items():
        #     if pwr is not None:
        #         print(f"🔋 Antenna {ant}: {pwr} dBm")
        #     else:
        #         print(f"⚠️ Antenna {ant}: not responding")

        #======================


        #======================



        # # === (3) Set Power for Antenna 1 ===
        # print("\n⚙️ Setting power for Antenna 1 to 28 dBm...")
        # if reader.set_power(ant=1, power=28, persist=True):
        #     print("✅ Power set successfully.")
        # else:
        #     print("❌ Failed to set power.")

        # print("\n🚀 Starting inventory for 10 seconds...")
        # reader.start_inventory()

        # # Let inventory run
        # time.sleep(5)


        # print("🛑 Stopping inventory...")
        # reader.stop_inventory()
        # print("✅ Inventory session complete.")

    else:
        print("❌ Initialization failed.")

    reader.close()
    print("🔌 Reader connection closed.")

if __name__ == "__main__":
    main()
