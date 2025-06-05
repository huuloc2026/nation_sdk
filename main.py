from nation_sdk import NationReader
import time

def main():
    port = "/dev/ttyUSB0"
    baudrate = 115200
    reader = NationReader(port, baudrate=baudrate)

    continue_inventory = True  # Boolean flag to control the inventory process

    try:
        # 1️⃣ Connect to the reader
        reader.connect()

        # 2️⃣ Optional: Stop the reader to ensure it is idle
        reader.stop()

        # 3️⃣ Query device information (optional)
        # reader.query_info()

        # 4️⃣ Query baseband version (optional)
        # reader.query_baseband_version()

        # 5️⃣ Query current baudrate
        reader.query_baudrate()

        # 6️⃣ Set antenna power (example: Antenna 1 at 30 dBm)
        reader.set_power({
            1: 20,  # Example: Antenna 1 at 20 dBm
        }, persistent=True)

        # 7️⃣ Start inventory
        print("🛰️ Starting inventory... Waiting for RFID tags.")
        reader.start_inventory()

        # 8️⃣ Continuously read EPC tags in a loop, based on continue_inventory flag
        while continue_inventory:
            reader.read_epc_tag() # Keep reading EPC tags

            # Check the condition for stopping inventory (e.g., stop after some time or condition)
            # Example condition to stop inventory after 10 seconds
            time.sleep(10)  # Adjust the sleep time as needed
            continue_inventory = False  # Set to False to stop the inventory loop after 10 seconds
            print("⏹️ Stopping inventory after 10 seconds.")

    except KeyboardInterrupt:
        print("⏹️ Inventory stopped by user.")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # Clean up and close the connection
        reader.close()
        print("🔌 Connection closed.")

if __name__ == "__main__":
    main()
