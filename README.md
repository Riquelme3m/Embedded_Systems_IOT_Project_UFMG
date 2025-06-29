# Real-Time Automation Tank Control - UFMG

A real-time IoT automation system for monitoring and controlling environmental parameters using ESP32, FreeRTOS, MQTT, and Node-RED.

---

## 📐 Project Architecture

**System Overview:**

- **ESP32** reads temperature and humidity from a DHT11 sensor, controls an alarm LED and a servo motor (ventilation system).
- **ESP32** communicates with a local MQTT broker to publish sensor data and receive control commands.
- **Node-RED** (Node.js-based) acts as the backend and dashboard, subscribing to MQTT topics, visualizing data, and allowing user control.
- **Data Logging:** Sensor and actuator states are logged into a `.txt` file for persistence and analysis.
- **Frontend UI:** Built with Node-RED Dashboard for real-time monitoring and control.

**Architecture Diagram:**  
  
![System Architecture](Architecture.png)

---

## 🖥️ Node-RED Dashboard

The dashboard provides real-time visualization of sensor readings, actuator states, and user controls for the servo motor and alarm threshold.


![Node-RED Dashboard](HMI_Screen.png)

---

## ⚙️ Technology Stack

- **Microcontroller:** ESP32
- **Firmware:** C++ (PlatformIO)
- **RTOS:** FreeRTOS (multiple tasks for sensor reading, MQTT, alarm, servo, and state publishing)
- **Backend & Dashboard:** Node-RED (Node.js-based)
- **Messaging:** MQTT (Mosquitto broker)
- **Data Logging:** Plain text file (`database.txt`)
- **Development Environment:** PlatformIO (VS Code)
- **Sensors/Actuators:** DHT11, SG90 Servo, LED

---

## 🧩 FreeRTOS Tasks

The ESP32 firmware uses FreeRTOS to run multiple tasks in parallel:
- **Sensor Task:** Periodically reads temperature and humidity.
- **MQTT Task:** Handles MQTT connection, publishing sensor data, and receiving commands.
- **Alarm Task:** Controls the alarm LED based on temperature threshold.
- **Servo Task:** Controls the servo motor position based on MQTT commands.
- **Publish State Task:** Publishes actuator states at regular intervals.

---

## 📝 Data Logging

All sensor and actuator events are logged in a tab-separated `.txt` file (`database.txt`) for easy analysis and import into spreadsheets.

Example entry:
```
ESP32_01	temperature	27.1	°C	2025-06-29T00:57:48.400Z
```

---

## 🚀 How to Run

1. **ESP32 Firmware:**
   - Edit WiFi and MQTT settings in `wifi_config.h` and `mqtt_handler.h`.
   - Build and upload using PlatformIO.

2. **Node-RED:**
   - Install Node-RED and required dashboard nodes.
   - Import the provided flow.
   - Configure MQTT broker connection.
   - Set up file node to log data to `database.txt`.

3. **MQTT Broker:**
   - Use Mosquitto or any MQTT broker on your local network.

---

## ⚡ Configuration

- **WiFi:** Set SSID and password in `wifi_config.h`.
- **MQTT:** Set broker IP in `mqtt_handler.h`.
- **PlatformIO:** All dependencies managed via `platformio.ini`.

---

## 📂 File Structure

```
include/
  hardware_control.h
  freertos_tasks.h
  mqtt_handler.h
  wifi_config.h
src/
  main.cpp
  hardware_control.cpp
  freertos_tasks.cpp
  mqtt_handler.cpp
  wifi_config.cpp
database.txt
platformio.ini
```

---


## MQTT Topics

Below are the MQTT topics used in this project:

### ESP32 → MQTT Broker (Publish)
- `/sensors/ESP32_01/DTH11_temp`  
  Publishes temperature readings from the DHT11 sensor.
- `/sensors/ESP32_01/DTH11_humidity`  
  Publishes humidity readings from the DHT11 sensor.
- `/states/ESP32_01/alarmLED`  
  Publishes the current state of the alarm LED (ON/OFF).
- `/states/ESP32_01/servo_motor_01`  
  Publishes the current state of the servo motor (ON/OFF).

### ESP32 ← MQTT Broker (Subscribe)
- `esp/servo_control`  
  Receives commands to turn the servo motor ON or OFF.
- `esp/temp_threshold`  
  Receives commands to update the temperature threshold.

---

## Backend & Data Logging

**Node-RED** is used as the backend for this project. Node-RED subscribes to the MQTT topics listed above, processes the incoming data, and writes relevant sensor and actuator information into a `.txt` file for logging and analysis. There is no Node.js backend or PostgreSQL database in this setup.

---


