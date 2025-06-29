#include <Arduino.h>
#include <time.h>
#include "wifi_config.h"
#include "mqtt_handler.h"
#include "hardware_control.h"
#include "freertos_tasks.h"

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("ESP32 Real-Time Automation System Starting...");
  
  // Initialize hardware

   


  initializeHardware();
  Serial.println("Hardware initialized");
  
  // Test servo movement
  testServoMovement();
  
  // Connect to WiFi
  setup_wifi();
  
  // Configure time
  configTime(0, 0, "pool.ntp.org");
  Serial.println("NTP time configured");
  
  // Setup MQTT
  setupMQTT();
  Serial.println("MQTT configured");
  
  // Create FreeRTOS tasks
  createFreeRTOSTasks();
  
  Serial.println("System initialization complete");
  Serial.println("FreeRTOS scheduler started");
}

void loop() {
  // Only handle MQTT connection maintenance
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();  // Essential for MQTT to receive commands
  
  // Remove this line - let FreeRTOS tasks handle sensor publishing
  // publishSensorDataPeriodically();  // ← REMOVE THIS
  
  vTaskDelay(pdMS_TO_TICKS(100));  // Short delay
}