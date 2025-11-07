# IoT-coursework-Ashish-Bhanu-Kumar
ESP32-BASED IOT ENVIRONMENTAL MONITORING SYSTEM USING MQTT AND NODE-RED


# ESP32 IoT Environmental Monitoring & Safety Control System

This repository contains the source code, Node-RED flow, and documentation for an IoT-based environmental monitoring node developed as part of the **Internet of Things (B31TF)** coursework at **Heriot-Watt University**.

## System Overview

The ESP32 node continuously measures **Temperature**, **Humidity**, and **Dew Point** using a DHT11/22 sensor and publishes live telemetry over **MQTT** to the `test.mosquitto.org` broker. A **Node-RED dashboard** visualizes the data, performs trend analytics, and allows remote control of the node through MQTT commands.

Key features:
- DHT sensor data acquisition
- MQTT telemetry and command channel
- Node-RED dashboard visualization
- Manual Alarm Override with LED flashing alert
- Auto recovery and event logging
- Dew point calculation and anomaly detection

## Hardware Used
- ESP32 Dev Board
- DHT11 or DHT22 Sensor
- NeoPixel LED (WS2812)
- USB power supply

## Communication Flow
1. ESP32 → MQTT Telemetry → Node-RED (Sensor Data)
2. Node-RED → MQTT Command → ESP32 (LED Control / Override)
3. ESP32 → MQTT Events → Node-RED (Logs & Acknowledgements)

## Repository Contents
| File | Description |
|------|--------------|
| `esp32_env.ino` | ESP32 main firmware code |
| `NodeRED_Flow.json` | Node-RED flow for dashboard |
| `project_screenshot.png` | Working dashboard snapshot |
| `README.md` | Documentation |
| `LICENSE` | MIT License |

## Developed By
**Ashish Bhanu Kumar**  
MSc Robotics with Industrial Applications  
Heriot-Watt University, Edinburgh  
📧 mail.from.ashish@gmail.com  

## MQTT Topics
