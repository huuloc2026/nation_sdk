from nation_sdk import NationReader
import time

def main():
    port = "/dev/ttyUSB0"
    baudrate = 115200
    reader = NationReader(port, baudrate=baudrate)

    try:
        # 1️⃣ Connect to the reader
        # reader.connect()

        # Send the "Read EPC Tag" command
        antenna_ports = 0x01  # Use antenna 1
        read_mode = 0  # Single read mode
        reader.send_read_epc_tag_command(antenna_ports, read_mode)

        # Receive the notification and parse the EPC data
        # Assume `data` is the byte data received from the reader
        data = reader.receive_data()  # This function needs to be defined to receive the data
        parsed_data = reader.parse_inventory_dataV2(data)
        print(f"Parsed EPC Data: {parsed_data}")

        # 2️⃣ Optional: Stop the reader to ensure it is idle
        # reader.stop()

        # # 3️⃣ Query device information (optional)
        # # reader.query_info()

        # # 4️⃣ Query baseband version (optional)
        # # reader.query_baseband_version()

        # # 5️⃣ Query current baudrate
        # reader.query_baudrate()


        # # Set the power for antenna 1 to 30 dBm, antenna 2 to 25 dBm
        # power_settings = {
        #     1: 30,  # Antenna 1 at 30 dBm
        #     2: 25,  # Antenna 2 at 25 dBm
        # }

        # reader.set_antenna_power(power_settings, persistent=True)


        # # 7️⃣ Start inventory
        # print("🛰️ Starting inventory... Waiting for RFID tags.")
        # # reader.start_inventory()

        # # Simulated data frame with MID=0x00 for EPC tag upload

        # reader.read_epc_tag() # Keep reading EPC tags

        # print("⏹️ Stopping inventory after 10 seconds.")

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



