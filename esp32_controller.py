#!/usr/bin/env python3
"""
ESP32 MQTT Controller
Simple script to control your ESP32 data collector
"""

import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime

# MQTT Configuration
BROKER_HOST = "192.168.0.65"  # Your local IP address
BROKER_PORT = 1883

class ESP32Controller:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        # Subscribe to all ESP32 topics
        client.subscribe("/sensors/ESP32_01/+")
        client.subscribe("/states/ESP32_01/+")
        client.subscribe("/sensor_monitors")
        
    def on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode())
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            if "DTH11_temp" in topic:
                print(f"[{timestamp}] Temperature: {payload['value']}°C")
            elif "DTH11_humidity" in topic:
                print(f"[{timestamp}] Humidity: {payload['value']}%")
            elif "alarmLED" in topic:
                print(f"[{timestamp}] Alarm LED: {payload['value']}")
            elif "servo_motor" in topic:
                print(f"[{timestamp}] Servo Motor: {payload['value']}")
            else:
                print(f"[{timestamp}] {topic}: {payload}")
                
        except json.JSONDecodeError:
            print(f"Raw message from {msg.topic}: {msg.payload.decode()}")
    
    def connect(self):
        self.client.connect(BROKER_HOST, BROKER_PORT, 60)
        self.client.loop_start()
        
    def set_temperature_threshold(self, temp):
        """Set temperature threshold for alarm"""
        self.client.publish("esp/temp_threshold", str(temp))
        print(f"Set temperature threshold to {temp}°C")
        
    def control_servo(self, state):
        """Control servo motor (ON/OFF)"""
        self.client.publish("esp/servo_control", state.upper())
        print(f"Servo motor {state.upper()}")
        
    def monitor(self):
        """Monitor ESP32 data"""
        print("Monitoring ESP32 data... (Press Ctrl+C to stop)")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping monitor...")
            self.client.loop_stop()
            self.client.disconnect()

def main():
    controller = ESP32Controller()
    controller.connect()
    
    print("ESP32 MQTT Controller")
    print("Commands:")
    print("  1. Monitor data")
    print("  2. Set temperature threshold")
    print("  3. Control servo motor")
    print("  4. Exit")
    
    while True:
        choice = input("\nEnter choice (1-4): ").strip()
        
        if choice == "1":
            controller.monitor()
        elif choice == "2":
            temp = float(input("Enter temperature threshold: "))
            controller.set_temperature_threshold(temp)
        elif choice == "3":
            state = input("Enter servo state (ON/OFF): ").strip()
            controller.control_servo(state)
        elif choice == "4":
            print("Goodbye!")
            break
        else:
            print("Invalid choice")

if __name__ == "__main__":
    main()