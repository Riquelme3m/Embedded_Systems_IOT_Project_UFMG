# Embedded Systems IoT Project - UFMG

Real-time automation system built with ESP32, featuring temperature/humidity monitoring, servo control, and MQTT connectivity.

## MQTT Communication Routes

### Published Topics (Data from ESP32)

#### 1. Temperature Data
- **Topic**: `esp/temperature`
- **Data Type**: Float (JSON)
- **Format**: `{"temperature": 25.6}`
- **Description**: Current temperature reading from DHT sensor
- **Update Frequency**: Every 10 seconds

#### 2. Humidity Data
- **Topic**: `esp/humidity`
- **Data Type**: Float (JSON)
- **Format**: `{"humidity": 60.2}`
- **Description**: Current humidity reading from DHT sensor
- **Update Frequency**: Every 10 seconds

#### 3. System Status
- **Topic**: `esp/status`
- **Data Type**: String (JSON)
- **Format**: `{"status": "online", "uptime": 12345}`
- **Description**: ESP32 system status and uptime
- **Update Frequency**: On connection and periodically

#### 4. Alarm Notifications
- **Topic**: `esp/alarm`
- **Data Type**: String (JSON)
- **Format**: `{"alarm": "temperature_high", "value": 35.2, "threshold": 30.0}`
- **Description**: Temperature threshold alerts
- **Trigger**: When temperature exceeds configured threshold

### Subscribed Topics (Commands to ESP32)

#### 1. Servo Motor Control
- **Topic**: `esp/servo_control`
- **Data Type**: String
- **Commands**:
  - `ON` - Rotate servo to 180°
  - `OFF` - Rotate servo to 0°
- **Description**: Controls the servo motor position
- **Example**: Publish `ON` to `esp/servo_control` to turn servo ON

#### 2. Temperature Threshold Configuration
- **Topic**: `esp/temperature_threshold`
- **Data Type**: Float
- **Format**: `25.5`
- **Description**: Sets the temperature alarm threshold
- **Example**: Publish `30.0` to set threshold at 30°C

## Connection Information

### MQTT Broker Settings
- **Host**: Configure in `src/config.h`
- **Port**: 1883 (default)
- **Client ID**: `ESP32_DataCollector`
- **Username/Password**: Configure as needed

### WiFi Configuration
Configure your WiFi credentials in `src/config.h`:
```cpp
#define WIFI_SSID "your_wifi_name"
#define WIFI_PASSWORD "your_wifi_password"
```

## Quick Start for External Clients

### Subscribing to Data (Receiving sensor data)
```bash
# Subscribe to temperature readings
mosquitto_sub -h [broker_ip] -t "esp/temperature"

# Subscribe to humidity readings
mosquitto_sub -h [broker_ip] -t "esp/humidity"

# Subscribe to all ESP32 data
mosquitto_sub -h [broker_ip] -t "esp/#"
```

### Publishing Commands (Sending commands to ESP32)
```bash
# Turn servo motor ON (180°)
mosquitto_pub -h [broker_ip] -t "esp/servo_control" -m "ON"

# Turn servo motor OFF (0°)
mosquitto_pub -h [broker_ip] -t "esp/servo_control" -m "OFF"

# Set temperature threshold to 28°C
mosquitto_pub -h [broker_ip] -t "esp/temperature_threshold" -m "28.0"
```

## Hardware Components
- ESP32 Development Board
- DHT22 Temperature/Humidity Sensor
- Servo Motor (SG90 or similar)
- LED indicators
- Buzzer for alarms

## Features
- Real-time sensor data collection
- MQTT communication
- Temperature threshold alarms
- Remote servo control
- FreeRTOS task management
- Automatic WiFi reconnection
- MQTT reconnection handling
