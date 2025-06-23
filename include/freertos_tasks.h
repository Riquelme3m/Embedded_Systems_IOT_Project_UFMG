#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include <Arduino.h>

// Task handles
extern TaskHandle_t sensorTaskHandle;
extern TaskHandle_t mqttTaskHandle;
extern TaskHandle_t alarmTaskHandle;
extern TaskHandle_t publishStateTaskHandle;
extern TaskHandle_t servoTaskHandle;

// Shared variables (protected by semaphores)
extern float currentTemperature;
extern float currentHumidity;
extern SemaphoreHandle_t temperatureSemaphore;
extern SemaphoreHandle_t humiditySemaphore;
extern QueueHandle_t servoQueue;

// Task function declarations
void sensorTask(void *parameter);
void mqttTask(void *parameter);
void alarmTask(void *parameter);
void servoTask(void *parameter);
void publishStateTask(void *parameter);

// Task creation functions
void createFreeRTOSTasks();

#endif