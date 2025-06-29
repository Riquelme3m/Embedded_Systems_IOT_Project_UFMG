#include "hardware_control.h"

// Pin Definitions
const int alarmLED = 14;
const int servoPin = 25;
const int tempHumiditySensor = 26;

// Hardware objects
DHT dht(tempHumiditySensor, DHT11);
Servo myServo;

// Initialize sensors and actuators
void initializeHardware() {
  dht.begin();
  pinMode(alarmLED, OUTPUT);

  Serial.print("Attaching servo to pin ");
  Serial.println(servoPin);

  myServo.setPeriodHertz(50);  // Standard servo frequency
  myServo.attach(servoPin, 500, 2400);  // Pulse width for ESP32Servo

  Serial.println("Servo attached with 50Hz frequency and 500-2400μs pulse width");
  delay(100);

  myServo.write(0); // Start servo at 0°
  delay(1000);
  digitalWrite(alarmLED, LOW); // Ensure alarm LED is OFF

  Serial.println("Hardware initialization complete");
}

// Read temperature from DHT sensor
float readTemperature() {
  return dht.readTemperature();
}

// Read humidity from DHT sensor
float readHumidity() {
  return dht.readHumidity();
}

// Set alarm LED ON or OFF
void setAlarmLED(bool state) {
  digitalWrite(alarmLED, state ? HIGH : LOW);
}

// Return true if alarm LED is ON
bool getAlarmLEDState() {
  return digitalRead(alarmLED) == HIGH;
}

// Set servo to a specific angle (0-180°)
void setServoState(int angle) {
  Serial.print("Setting servo to: ");
  Serial.print(angle);
  Serial.println("°");
  myServo.write(angle);
  delay(500); // Allow time for servo to reach position
}

// Test servo movement at startup
void testServoMovement() {
  Serial.println("Testing servo movement...");
  Serial.println("Moving to 180°");
  myServo.write(180);
  delay(1000);
  Serial.println("Moving to 0°");
  myServo.write(0);
  delay(1000);
  Serial.println("Servo test complete");
}

// Get current servo angle, or -1 if not attached
int getServoState() {
  if (!myServo.attached()) {
    Serial.println("ERROR: Servo not attached when reading position!");
    return -1;
  }
  return myServo.read();
}