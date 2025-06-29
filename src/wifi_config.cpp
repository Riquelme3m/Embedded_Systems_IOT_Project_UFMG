#include "wifi_config.h"

// WiFi credentials
const char* ssid = "THIAGO";
const char* pswrd = "thiago@102030";

void setup_wifi() {
  delay(10); // Short delay before starting WiFi connection
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);
  WiFi.begin(ssid, pswrd);
  // Wait until WiFi is connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(" Status: ");
    Serial.println(WiFi.status());
  }
  Serial.println("\nWiFi connected successfully");
}

// Returns true if WiFi is connected
bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}