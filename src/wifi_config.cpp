#include "wifi_config.h"

const char* ssid = "THIAGO";
const char* pswrd = "thiago@102030";

void setup_wifi() {
  delay(10);
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);
  WiFi.begin(ssid, pswrd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(" Status: ");
    Serial.println(WiFi.status());
  }
  Serial.println("\nWiFi connected successfully");
}

bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}