#include "freertos_tasks.h"
#include "hardware_control.h"
#include "mqtt_handler.h"
#include "wifi_config.h"

// Task handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t mqttTaskHandle = NULL;
TaskHandle_t alarmTaskHandle = NULL;
TaskHandle_t publishStateTaskHandle = NULL;
TaskHandle_t servoTaskHandle = NULL;

// Shared variables
float currentTemperature = 0.0;
float currentHumidity = 0.0;
SemaphoreHandle_t temperatureSemaphore = NULL;
SemaphoreHandle_t humiditySemaphore = NULL;
QueueHandle_t servoQueue = NULL;

// External variables from mqtt_handler
extern float temperatureThreshold;

// Task: Read temperature and humidity periodically
void sensorTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2000); 
  
  for (;;) {
    float temp = readTemperature();
    float humidity = readHumidity();
    if (!isnan(temp) && !isnan(humidity)) {
      // Update shared variables with semaphore protection
      if (xSemaphoreTake(temperatureSemaphore, portMAX_DELAY) == pdTRUE) {
        currentTemperature = temp;
        xSemaphoreGive(temperatureSemaphore);
      }
      if (xSemaphoreTake(humiditySemaphore, portMAX_DELAY) == pdTRUE) {
        currentHumidity = humidity;
        xSemaphoreGive(humiditySemaphore);
      }
      Serial.print("Sensor Task - Temperature: ");
      Serial.print(temp);
      Serial.print("°C, Humidity: ");
      Serial.print(humidity);
      Serial.println("%");
    } else {
      Serial.println("Sensor Task - Failed to read from DHT sensor");
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task: Handle MQTT connection and publish sensor data
void mqttTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2000); // 2 seconds
  
  for (;;) {
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();
    float temp, humidity;
    if (xSemaphoreTake(temperatureSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      temp = currentTemperature;
      xSemaphoreGive(temperatureSemaphore);
      if (temp != 0.0) {
        publishTemperatureSensorData(temp);
        Serial.println("MQTT Task - Published temperature data");
      }
    }
    if (xSemaphoreTake(humiditySemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      humidity = currentHumidity;
      xSemaphoreGive(humiditySemaphore);
      if (humidity != 0.0) {
        publishHumiditySensorData(humidity);
        Serial.println("MQTT Task - Published humidity data");
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task: Control alarm LED based on temperature threshold
void alarmTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 second
  
  for (;;) {
    float temp;
    if (xSemaphoreTake(temperatureSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      temp = currentTemperature;
      xSemaphoreGive(temperatureSemaphore);
      if (temp > temperatureThreshold) {
        setAlarmLED(true);
        Serial.println("Alarm Task - Temperature alarm ON");
      } else {
        setAlarmLED(false);
        Serial.println("Alarm Task - Temperature alarm OFF");
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task: Publish actuator states (alarm, servo) periodically
void publishStateTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500); // 500ms
  
  for (;;) {
    if (client.connected()) {
      publishAlarmState();
      publishServoMotorState();
      Serial.println("State Task - Published actuator states");
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task: Set servo position based on queue input
void servoTask(void *parameter) {
  int newPosition;
  for (;;) {
    // Wait for a new position from the queue
    if (xQueueReceive(servoQueue, &newPosition, portMAX_DELAY) == pdTRUE) {
      setServoState(newPosition);
      Serial.print("Servo Task - Position set to: ");
      Serial.println(newPosition);
    }
  }
}

// Create all FreeRTOS tasks and synchronization primitives
void createFreeRTOSTasks() {
  temperatureSemaphore = xSemaphoreCreateMutex();
  humiditySemaphore = xSemaphoreCreateMutex();
  servoQueue = xQueueCreate(5, sizeof(int));
  
  if (temperatureSemaphore == NULL || humiditySemaphore == NULL || servoQueue == NULL) {
    Serial.println("Failed to create semaphores or queue");
    return;
  }
  
  xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 2, &sensorTaskHandle);
  xTaskCreate(mqttTask, "MQTTTask", 8192, NULL, 4, &mqttTaskHandle);
  xTaskCreate(alarmTask, "AlarmTask", 2048, NULL, 1, &alarmTaskHandle);
  xTaskCreate(publishStateTask, "PublishStateTask", 4096, NULL, 1, &publishStateTaskHandle);
  xTaskCreate(servoTask, "ServoTask", 2048, NULL, 2, &servoTaskHandle);
  
  Serial.println("FreeRTOS tasks created successfully");
}