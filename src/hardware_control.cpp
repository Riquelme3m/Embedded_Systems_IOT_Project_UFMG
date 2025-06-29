#include "hardware_control.h"

// Pin Definitions
const int alarmLED = 14;
const int servoPin = 25;  // Changed from 27 to 25
const int tempHumiditySensor = 26;

// Hardware objects
DHT dht(tempHumiditySensor, DHT11);
Servo myServo;

void initializeHardware() {
  dht.begin();
  pinMode(alarmLED, OUTPUT);
  
  // ESP32Servo initialization matching working code
  Serial.print("Attaching servo to pin ");
  Serial.println(servoPin);
  
  myServo.setPeriodHertz(50);  // Set frequency to 50 Hz (standard for servos)
  myServo.attach(servoPin, 500, 2400);  // Use pulse widths from working code
  
  Serial.println("Servo attached with 50Hz frequency and 500-2400μs pulse width");
  delay(100);
  
  myServo.write(0); // Start at 0 position like working code
  delay(1000);
  digitalWrite(alarmLED, LOW); // Initialize LED to OFF
  
  Serial.println("Hardware initialization complete");
}

float readTemperature() {
  return dht.readTemperature();
}

float readHumidity() {
  return dht.readHumidity();
}

void setAlarmLED(bool state) {
  digitalWrite(alarmLED, state ? HIGH : LOW);
}

bool getAlarmLEDState() {
  return digitalRead(alarmLED) == HIGH;
}



void setServoState(int angle) {
  Serial.print("Setting servo to: ");
  Serial.print(angle);
  Serial.println("°");
  myServo.write(angle);
  delay(500); // Allow time for servo to reach position
}
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

int getServoState() {
  if (!myServo.attached()) {
    Serial.println("ERROR: Servo not attached when reading position!");
    return -1;
  }
  return myServo.read();
}