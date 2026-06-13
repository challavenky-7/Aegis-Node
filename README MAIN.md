**Creators: Venkateswar Challa & Kuruba Saranya Sai Sree**




# Aegis Node — Stable Firmware v5.0

An advanced, non-blocking ESP32-based environmental and health monitoring hub. This system interfaces with multiple I2C and analog sensors to track biometrics and gas density, streaming metrics in real-time over an asynchronous web server.

## 🚀 Key Improvements in v5.0
* **Watchdog Loop Patches**: Replaced all blocking `delay()` calls with non-blocking `millis()` timers to prevent MAX30100 FIFO overflow and unexpected reboots.
* **Network Stability**: Refactored the WiFi auto-reconnect behavior to prevent brownout loops and network starvation.
* **Optimized Memory**: Entirely stack-allocated design eliminating runtime heap fragmentation.

## 🛠️ Hardware Wiring Guide
| Component | Pin (ESP32) | Protocol |
| :--- | :--- | :--- |
| **I2C SDA** | GPIO 21 | Shared I2C Bus |
| **I2C SCL** | GPIO 22 | Shared I2C Bus |
| **MQ-2 Analog Out** | GPIO 34 | ADC1 (WiFi-safe channel) |
| **Active Buzzer (+)** | GPIO 25 | DAC1 / Digital Out |
| **All Modules VCC** | 3.3V | Main Power Rail |

## 📦 Required Libraries
Install these via the Arduino Library Manager:
1. `MAX30100lib` (by OXullo Intersecans)
2. `Adafruit MPU6050`
3. `Adafruit SSD1306`
4. `Adafruit BusIO`
5. `ESPAsyncWebServer` (via GitHub)
6. `AsyncTCP` (via GitHub)
