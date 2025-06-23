#ifndef HARDWARE_CONTROL_H
#define HARDWARE_CONTROL_H

#include <DHT.h>
#include <ESP32Servo.h>

// Pin Definitions
extern const int alarmLED;
extern const int servoPin;
extern const int tempHumiditySensor;

// Hardware objects
extern DHT dht;
extern Servo myServo;

// Function declarations
void initializeHardware();
float readTemperature();
float readHumidity();
void setAlarmLED(bool state);
bool getAlarmLEDState();
void setServoState(int angle);
void testServoMovement(); // Add test function declaration
int getServoState();

#endif