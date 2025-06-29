#include "mqtt_handler.h"
#include "hardware_control.h"
#include "freertos_tasks.h"
#include <time.h>

// MQTT Configuration
const char* mqtt_server = "192.168.0.65";
const char* machine_id = "ESP32_01";

// Sensor and Actuator Metadata
const char* sensorTempId = "DTH11_temp";
const char* humiditySensorId = "DTH11_humidity";
const char* alarmLedId = "alarmLED";
const char* servoMotorId = "servo_motor_01";

const char* sensorTempDataType = "Temperature °C";
const char* sensorHumidityDataType = "Humidity";
const char *alarmLedDataType = "ON/OFF";
const char* servoMotorDataType = "ON/OFF";

// Global MQTT objects
WiFiClient espClient;
PubSubClient client(espClient);

// State variables
float temperatureThreshold = 30.0;
bool initialInfoPublished = false;

// Sensor publish interval (ms)
const unsigned long sensorInterval = 1000;

// Returns current time as ISO8601 string
String getISO8601Timestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "1970-01-01T00:00:00Z";
  }
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}

// Configure MQTT server and callback
void setupMQTT() {
  client.setServer(mqtt_server, 1883);
  client.setKeepAlive(60);
  client.setCallback(mqttCallback);
}

// Publish metadata about sensors and actuators
void publishInitialSensorInfo() {
  JsonDocument doc;
  doc["machine_id"] = machine_id;

  JsonArray sensors = doc["sensors"].to<JsonArray>();
  JsonArray actuators = doc["actuators"].to<JsonArray>();

  JsonObject sensor1 = sensors.add<JsonObject>();
  sensor1["sensor_id"] = sensorTempId;
  sensor1["data_type"] = sensorTempDataType;
  sensor1["data_interval"] = sensorInterval;

  JsonObject sensor2 = sensors.add<JsonObject>();
  sensor2["sensor_id"] = humiditySensorId;
  sensor2["data_type"] = sensorHumidityDataType;
  sensor2["data_interval"] = sensorInterval;

  JsonObject sensor3 = sensors.add<JsonObject>();
  sensor3["sensor_id"] = alarmLedId;
  sensor3["data_type"] = alarmLedDataType;
  sensor3["data_interval"] = "No specified data interval";

  JsonObject actuator = actuators.add<JsonObject>();
  actuator["actuator_id"] = servoMotorId;
  actuator["data_type"] = servoMotorDataType;
  actuator["data_interval"] = "No specified data interval";

  char buffer[512];
  size_t n = serializeJson(doc, buffer);
  client.publish("/sensor_monitors", buffer, n);
  Serial.println("Initial sensor info published:");
  Serial.println(buffer);
}

// Publish temperature sensor data to MQTT
void publishTemperatureSensorData(float temp) {
  JsonDocument doc;
  doc["timestamp"] = getISO8601Timestamp();
  doc["value"] = temp;
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  client.publish("/sensors/ESP32_01/DTH11_temp", buffer, n);
}

// Publish humidity sensor data to MQTT
void publishHumiditySensorData(float humidity) {
  JsonDocument doc;
  doc["timestamp"] = getISO8601Timestamp();
  doc["value"] = humidity;
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  client.publish("/sensors/ESP32_01/DTH11_humidity", buffer, n);
}

// Publish alarm LED state to MQTT
void publishAlarmState() {
  JsonDocument doc;
  doc["timestamp"] = getISO8601Timestamp();
  doc["value"] = getAlarmLEDState() ? "ON" : "OFF";
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  client.publish("/states/ESP32_01/alarmLED", buffer, n);
}

// Publish servo motor state to MQTT
void publishServoMotorState() {
  JsonDocument doc;
  doc["timestamp"] = getISO8601Timestamp();
  doc["value"] = getServoState() > 90 ? "ON" : "OFF";
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  client.publish("/states/ESP32_01/servo_motor_01", buffer, n);
}

// (Legacy) Periodically publish sensor and actuator data if called in loop
void publishSensorDataPeriodically() {
    static unsigned long lastSensorRead = 0;
    unsigned long currentTime = millis();
    // Publish sensor data every sensorInterval ms
    if (currentTime - lastSensorRead >= sensorInterval) {
        if (client.connected()) {
            float temp = readTemperature();
            if (!isnan(temp)) {
                publishTemperatureSensorData(temp);
                Serial.print("Published temperature: ");
                Serial.println(temp);
                // Set alarm LED based on threshold
                if (temp > temperatureThreshold) {
                    setAlarmLED(true);
                } else {
                    setAlarmLED(false);
                }
                publishAlarmState();
            }
            float humidity = readHumidity();
            if (!isnan(humidity)) {
                publishHumiditySensorData(humidity);
                Serial.print("Published humidity: ");
                Serial.println(humidity);
            }
            publishServoMotorState();
            lastSensorRead = currentTime;
        }
    }
}

// Handles incoming MQTT messages for threshold and servo control
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Received on topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);

  // Handle temperature threshold update
  if (strcmp(topic, "esp/temp_threshold") == 0) {
    temperatureThreshold = message.toFloat();
    Serial.print("Updated temperature threshold to: ");
    Serial.println(temperatureThreshold);
    // Immediately update alarm LED based on new threshold
    if (xSemaphoreTake(temperatureSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      float currentTemp = currentTemperature;
      xSemaphoreGive(temperatureSemaphore);
      if (currentTemp != 0.0) {
        if (currentTemp > temperatureThreshold) {
          setAlarmLED(true);
          Serial.println("Callback - Alarm turned ON immediately");
        } else {
          setAlarmLED(false);
          Serial.println("Callback - Alarm turned OFF immediately");
        }
        // State will be published by FreeRTOS task
      }
    }
  }

  // Handle servo control command
  if (strcmp(topic, "esp/servo_control") == 0) {
    if (message == "ON") {
        setServoState(180);
        Serial.println("Callback - Servo turned ON");
    } else if (message == "OFF") {
        setServoState(0);
        Serial.println("Callback - Servo turned OFF");
    }
    // Immediately publish new servo state
    publishServoMotorState();
    Serial.println("Callback - Published servo state immediately");
  }
}

// Attempt to reconnect to MQTT broker and subscribe to topics
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client_01")) {
      Serial.println("connected");
      client.subscribe("esp/temp_threshold");
      client.subscribe("esp/servo_control");
      Serial.println("Subscribed");
      if (!initialInfoPublished) {
        publishInitialSensorInfo();
        initialInfoPublished = true;
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}