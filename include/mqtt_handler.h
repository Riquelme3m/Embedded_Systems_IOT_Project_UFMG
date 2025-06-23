#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// MQTT Configuration
extern const char* mqtt_server;
extern const char* machine_id;

// MQTT Client
extern WiFiClient espClient;
extern PubSubClient client;

// Temperature threshold for alarm
extern float temperatureThreshold;

// Function declarations
void setupMQTT();
void reconnectMQTT();
void publishInitialSensorInfo();
void publishTemperatureSensorData(float temp);
void publishHumiditySensorData(float humidity);
void publishAlarmState();
void publishServoMotorState();
void mqttCallback(char* topic, byte* payload, unsigned int length);
String getISO8601Timestamp();

#endif