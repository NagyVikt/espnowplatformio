ESP-Now Master-Slave Network with Jetson Companion
🚀 Project Overview

This project implements a scalable, one-to-many ESP-Now network designed for reliable, master-orchestrated communication. It consists of a single Master node (ESP32-C6), multiple Slave nodes (ESP32 variants), and a Jetson Companion for high-level control, data logging, and management.

Communication is strictly hierarchical: Slaves only communicate with the Master upon request and never with each other. This architecture is ideal for applications requiring deterministic, low-power, and scalable wireless messaging.
Key Use Cases

    Sensor Farms: The Master periodically polls a large number of Slaves equipped with environmental sensors. The data is aggregated and forwarded to the Jetson for real-time logging and analysis.

    Distributed Actuator Networks: The Jetson computes control strategies and sends commands to the Master, which then dispatches them to specific Slaves to control motors, lights, or other actuators.

    Deterministic Low-Power Messaging: By ensuring only one Slave communicates at a time, the network guarantees predictable message timing and allows Slaves to optimize their power consumption by sleeping between communication windows.

🏗️ System Architecture

The system is composed of three main components: the Jetson Companion, the Master ESP32, and a fleet of Slave ESP32 devices.

┌──────────────────┐      USB-Serial      ┌──────────────┐
│ Jetson Companion │◀────────────────────▶│ Master (ESP32) │
│ (Manages & Logs) │                      │ (Orchestrator) │
└──────────────────┘                      └──────┬───────┘
                                                 │
                                           ESP-Now (2.4GHz)
                                                 │
                           ┌─────────────────────┼─────────────────────┐
                           │                     │                     │
                      ┌────┴────┐           ┌────┴────┐           ┌────┴────┐
                      │ Slave 1 │           │ Slave 2 │           │ Slave N │
                      └─────────┘           └─────────┘           └─────────┘

    Jetson Companion

        Connects to the Master via USB-Serial (e.g., /dev/ttyUSB0).

        Maintains the master list of all Slave MAC addresses.

        Sends the MAC address list to the Master on startup or upon request.

        Issues high-level commands (e.g., "start polling", "send command to Slave X").

        Receives and logs all data forwarded by the Master.

    Master (ESP32-C6)

        Acts as the central orchestrator of the ESP-Now network.

        Receives the list of Slave MAC addresses from the Jetson.

        Initializes ESP-Now and adds each Slave as a peer.

        Polls one Slave at a time in a round-robin fashion.

        Forwards every response from a Slave to the Jetson over the serial connection.

    Slaves (ESP32)

        Remain in a low-power listening mode until addressed by the Master.

        Never initiate communication.

        Upon receiving a poll request, they perform a predefined task (e.g., read a sensor, actuate a component).

        Send a unicast response containing the result directly back to the Master.

🔌 Communication Protocol
1. Initialization & MAC List Distribution

    Jetson → Master (Serial): The Jetson initiates the process by sending the complete list of Slave MAC addresses to the Master, prefixed for easy parsing.

    MAC_LIST:AA:BB:CC:DD:EE:01,AA:BB:CC:DD:EE:02,...\n

2. Master-Slave Polling Loop

    Master → Slave (ESP-Now): The Master sends a simple, 1-byte poll command to a single Slave.

        Payload: [ 0x01 ] (where 0x01 is CMD_POLL).

    Slave → Master (ESP-Now): The Slave responds with its data, for example, sensor readings.

        Payload: struct SensorData { float temperature; float humidity; };

    Master → Jetson (Serial): The Master wraps the Slave's data and its MAC address in a prefixed string and forwards it to the Jetson.

    DATA:AA:BB:CC:DD:EE:01,25.5,60.2\n

    Error Handling: If a Slave doesn't respond within a defined timeout, the Master sends an error message to the Jetson.

    TIMEOUT:AA:BB:CC:DD:EE:01\n

✅ Developer Task Checklist

Here are the specific implementation tasks for each component.
👨‍💻 Jetson Companion (Python/Node.js)

    [ ] Task 1: Serial Port Management

        Implement a function to automatically find and connect to the Master ESP32 (e.g., scan /dev/ttyUSB*).

        Open the serial port with the correct settings (115200 baud, 8N1).

        Implement robust read/write logic with error handling.

    [ ] Task 2: MAC Address Management

        Store the list of Slave MAC addresses in a configuration file (e.g., slaves.json).

        On startup, read the file and send the MAC_LIST:... command to the Master.

    [ ] Task 3: Data Logging and Parsing

        Implement a serial data listener that runs in a separate thread/process.

        Parse incoming messages from the Master (e.g., DATA:..., TIMEOUT:...).

        Log parsed data to a CSV file or a database with timestamps.

    [ ] Task 4: Command Interface (Optional)

        Create a simple command-line interface (CLI) or a basic UI to send commands to the Master (e.g., rescan, pause, send_to_slave <MAC> <command>).

🛠️ Master ESP32 (PlatformIO/ESP-IDF)

    [ ] Task 1: Setup WiFi and ESP-Now

        Initialize NVS, esp_netif, and WiFi in Station (STA) mode.

        Initialize the ESP-Now service.

        Register the esp_now_register_send_cb() and esp_now_register_recv_cb() callback functions.

    [ ] Task 2: Serial Communication with Jetson

        Implement a setupSerial() function.

        Create a task serialListenerTask to handle incoming messages from the Jetson.

        Parse the MAC_LIST:... command. Store the MAC addresses in an array or vector.

    [ ] Task 3: Peer Management

        After receiving the MAC list, iterate through it and add each Slave as an ESP-Now peer using esp_now_add_peer().

        Ensure the peer configuration specifies the correct Wi-Fi channel (e.g., channel 1) and no encryption (for simplicity, can be enabled later).

    [ ] Task 4: Polling Loop

        Create a pollingTask that iterates through the list of registered Slave peers.

        For each peer, send a CMD_POLL message using esp_now_send().

        Use a semaphore or event group to wait for the onDataRecv callback to signal that a response has arrived or a timeout has occurred.

        If data is received, format and forward it to the Jetson via Serial.

        If a timeout occurs, report the failure to the Jetson.

        Include a configurable delay (INTER_POLL_DELAY) between polling each Slave.

🤖 Slave ESP32 (PlatformIO/ESP-IDF)

    [ ] Task 1: Setup WiFi and ESP-Now

        Initialize NVS, esp_netif, and WiFi in Station (STA) mode.

        Initialize the ESP-Now service.

        Register only the esp_now_register_recv_cb() callback (onSlaveRecv).

    [ ] Task 2: Identify Own MAC Address

        On startup, get the device's own MAC address using esp_wifi_get_mac() and print it to the serial monitor. This is crucial for adding it to the Jetson's slaves.json file.

    [ ] Task 3: Implement Receive Handler (onSlaveRecv)

        Inside the callback, first check that the message is from the Master (optional, but good practice).

        Check if the received data corresponds to CMD_POLL.

        If it is a poll command, proceed to the next task.

    [ ] Task 4: Sensor/Actuator Logic

        Implement the device's primary function (e.g., read_dht_sensor(), toggle_led()).

        Populate a response struct with the resulting data.

    [ ] Task 5: Send Response

        Use esp_now_send() to send the response struct back to the sender's address (info->src_addr).

    [ ] Task 6: Power Management (Optional)

        Implement light or deep sleep logic to conserve power between expected polls. The Master's polling interval will determine the optimal sleep duration.

🛠️ Setup & Build Instructions

    Hardware Connections:

        Connect the Master ESP32 to the Jetson using a reliable Micro-USB data cable.

    Software Installation:

        Install Visual Studio Code with the PlatformIO IDE extension.

        Clone this repository to your local machine.

    Project Configuration:

        Open the project folder in VS Code. PlatformIO should automatically recognize it.

        On the Jetson, create the slaves.json file and populate it with the MAC addresses of your slave devices.

    Build & Flash:

        For Slaves:

            Connect a Slave ESP32.

            Run the "Upload" command in PlatformIO for the slave environment.

            Open the Serial Monitor to get its MAC address. Add it to slaves.json.

        pio run -e slave -t upload
        pio run -e slave -t monitor

        For Master:

            Connect the Master ESP32.

            Run the "Upload" command in PlatformIO for the master environment.

        pio run -e master -t upload

    Run the System:

        Start the Python/Node.js script on the Jetson. It should connect to the Master and send the MAC list.

        The Master will begin polling the Slaves, and you should see DATA: or TIMEOUT: messages appearing in the Jetson's log.

💡 Tips & Best Practices

    Serial Framing: Use clear, newline-terminated (\n) prefixes (MAC_LIST:, DATA:, ERR:) for all serial communication. This makes parsing on both ends much more reliable.

    Channel Locking: For improved performance, ensure both the Master and all Slaves are configured to operate on the same, fixed Wi-Fi channel.

    Timeouts and Retries: The Master should implement a retry mechanism (e.g., 2-3 retries) for a timed-out Slave before marking it as offline and moving to the next.

    Scalability: Avoid using String objects in the ESP32 code. Use character arrays for fixed-size buffers to prevent memory fragmentation, especially on the Master which handles many peers.

    Security: For production environments, enable ESP-Now encryption (esp_now_peer_info_t.encrypt = true;) and provision a Primary Master Key (PMK) on all devices.