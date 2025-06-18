from nation_sdk import NationReader
import time

def main():

    port = "/dev/ttyUSB0"
    braud = 115200
    reader = NationReader(port, braud)
    reader.open()

    print("🔧 Connecting and initializing reader...")
    if reader.Connect_Reader_And_Initialize():
        info = reader.Query_Reader_Information()
        print("📡 Reader Info:")
        for k, v in info.items():
            print(f"  {k}: {v}")
        print("\n🔍 Querying antenna power levels...")
        powers = reader.query_reader_power()
        for ant in range(1, 5):
            val = powers.get(ant)
            if val is not None:
                print(f"  🔧 Antenna {ant}: {val} dBm")
            else:
                print(f"  ⚠️ Antenna {ant}: N/A")


        # reader.set_power(power=32,persist=False)
    # # # === POWER QUERY ===
    #     try:

    #         powers = reader.get_power()

    #         if powers:
    #             print("🔋 Antenna Power Levels:")
    #             for ant_id, dbm in powers.items():
    #                 print(f"  🔧 Antenna {ant_id}: {dbm} dBm")
    #         else:
    #             print("⚠️ No power settings returned.")

    #     except Exception as e:
    #         print(f"❌ Error during power query: {e}")
     

        

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



        # # # === (3) Set Power for Antenna 1 ===
        # print("\n⚙️ Setting power for Antenna 1 to 28 dBm...")
        # if reader.set_power(ant=1, power=5, persist=True):
        #     print("✅ Power set successfully.")
        # else:
        #     print("❌ Failed to set power.")

        print("\n🚀 Starting inventory for 10 seconds...")
        reader.start_inventory()

        # Let inventory run
        time.sleep(10)


        print("🛑 Stopping inventory...")
        reader.stop_inventory()
        print("✅ Inventory session complete.")

    else:
        print("❌ Initialization failed.")

    reader.close()
    print("🔌 Reader connection closed.")

if __name__ == "__main__":
    main()