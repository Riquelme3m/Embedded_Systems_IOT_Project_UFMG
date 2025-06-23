#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <WiFi.h>

// WiFi Configuration
extern const char* ssid;
extern const char* pswrd;

// Function declarations
void setup_wifi();
bool isWiFiConnected();

#endif