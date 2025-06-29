#include <Arduino.h>
#include <time.h>
#include "wifi_config.h"
#include "mqtt_handler.h"
#include "hardware_control.h"
#include "freertos_tasks.h"

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Real-Time Automation System Starting...");
  
  // Initialize hardware (sensors, actuators ...)
  initializeHardware();
  Serial.println("Hardware initialized");
  
  // Test servo movement at startup
  testServoMovement();
  
  // Connect to WiFi
  setup_wifi();
  
  // Configure NTP time
  configTime(0, 0, "pool.ntp.org");
  Serial.println("NTP time configured");
  
  // Setup MQTT connection and callbacks
  setupMQTT();
  Serial.println("MQTT configured");
  
  // Create FreeRTOS tasks for multitasking
  createFreeRTOSTasks();
  
  Serial.println("System initialization complete");
  Serial.println("FreeRTOS scheduler started");
}

void loop() {
  // Maintain MQTT connection and process incoming messages
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Sensor publishing is handled by FreeRTOS tasks

  vTaskDelay(pdMS_TO_TICKS(100));  // Short delay to yield to other tasks
}