# Orbit Board Firmware
**Orbit** is a custom flight data-logging and telemetry board designed for rocket missions at **AC x KMUTT Rocket Camp 2025**.  
This firmware is responsible for collecting sensor data, transmitting real-time telemetry via LoRa, and storing flight data on an SD card.

## Overview
The Orbit board gathers and transmits data from multiple sensors during flight:
| Component | Function |
|------------|-----------|
| **LIS331DLHTR** | 3-axis accelerometer for motion and acceleration measurement |
| **BME280** | Temperature, humidity, and pressure sensor |
| **LC86G** | GNSS (GPS) module for position tracking |
| **LoRa E22-900M22S** | Long-range wireless module for downlink telemetry |
| **SD Card** | Data logging and storage |

All data is processed by the onboard microcontroller and can be transmitted or stored for post-flight analysis.
