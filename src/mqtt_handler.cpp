#include "mqtt_handler.h"
#include "hardware_control.h"
#include "freertos_tasks.h"
#include <time.h>

// MQTT Configuration
const char* mqtt_server = "192.168.0.65";  // Your local IP address
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

// Global Objects
WiFiClient espClient;
PubSubClient client(espClient);

// State
float temperatureThreshold = 30.0;
bool initialInfoPublished = false;

// Sensor intervals
const unsigned long sensorInterval = 5000;

String getISO8601Timestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "1970-01-01T00:00:00Z";
  }
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}

void setupMQTT() {
  client.setServer(mqtt_server, 1883);
  client.setKeepAlive(60);
  client.setCallback(mqttCallback);
}

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

void publishTemperatureSensorData(float temp) {
  JsonDocument doc;
  doc["timestamp"] = getISO8601Timestamp();
  doc["value"] = temp;
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  client.publish("/sensors/ESP32_01/DTH11_temp", buffer, n);
}

void publishHumiditySensorData(float humidity) {
  JsonDocument doc;
  doc["timestamp"] = getISO8601Timestamp();
  doc["value"] = humidity;
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  client.publish("/sensors/ESP32_01/DTH11_humidity", buffer, n);
}

void publishAlarmState() {
  JsonDocument doc;
  doc["timestamp"] = getISO8601Timestamp();
  doc["value"] = getAlarmLEDState() ? "ON" : "OFF";
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  client.publish("/states/ESP32_01/alarmLED", buffer, n);
}

void publishServoMotorState() {
  JsonDocument doc;
  doc["timestamp"] = getISO8601Timestamp();
  doc["value"] = getServoState() > 90 ? "ON" : "OFF";
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  client.publish("/states/ESP32_01/servo_motor_01", buffer, n);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Received on topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);

  if (strcmp(topic, "esp/temp_threshold") == 0) {
    temperatureThreshold = message.toFloat();
    Serial.print("Updated temperature threshold to: ");
    Serial.println(temperatureThreshold);
  } 
  else if (strcmp(topic, "esp/servo_control") == 0) {
    int newPosition = -1;
    if (message.equalsIgnoreCase("ON")) {
      Serial.println("MQTT: Received servo command ON");
      newPosition = 180;
    } 
    else if (message.equalsIgnoreCase("OFF")) {
      Serial.println("MQTT: Received servo command OFF");
      newPosition = 0;
    }

    if (newPosition != -1) {
      if (xQueueSend(servoQueue, &newPosition, portMAX_DELAY) != pdPASS) {
        Serial.println("Failed to send to servo queue");
      } else {
        Serial.print("MQTT: Sent position to servo queue: ");
        Serial.println(newPosition);
      }
    }
  }
}

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