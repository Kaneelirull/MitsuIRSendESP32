# MitsuIRSendESP32

A robust ESP32-based IR controller for Mitsubishi air conditioners, integrated with MQTT and Home Assistant. This project allows you to send IR commands to your AC unit, control various settings like mode, fan speed, vane positions, and temperature, and receive state updates over MQTT. Includes support for OTA updates and time-based weekly reboots.

---

## Features

* IR control of Mitsubishi A/C using `IRremoteESP8266`
* Full Home Assistant MQTT discovery support
* Supports commands for:

  * Power on/off
  * HVAC modes (cool, heat, dry, fan\_only, auto)
  * Temperature (16°C to 30°C)
  * Fan speeds (auto to maxmax)
  * Vertical vane position
  * Left horizontal vane
  * Wide vane swing settings
* Button2 support for physical toggle
* OTA updates via `ArduinoOTA`
* Weekly scheduled reboot for stability
* Debug logging

---

## Hardware Requirements

* ESP32 board
* IR LED connected to GPIO 25
* Button connected to GPIO 39 (with pull-up)

---

## Installation

1. Clone the repo:

   ```bash
   git clone https://github.com/Kaneelirull/MitsuIRSendESP32.git
   ```

2. Install libraries in Arduino IDE:

   * `IRremoteESP8266`
   * `Button2`
   * `PubSubClient`
   * `ArduinoJson`
   * `ArduinoOTA`

3. Set Wi-Fi and MQTT credentials in the sketch:

   ```cpp
   const char* WIFI_SSID = "your-ssid";
   const char* WIFI_PASSWORD = "your-password";
   const char* MQTT_SERVER = "your-mqtt-server";
   const char* MQTT_USER = "your-mqtt-user";
   const char* MQTT_PASS = "your-mqtt-password";
   ```

4. Upload to ESP32 and monitor serial output at 115200 baud.

---

## MQTT Topics

### Commands

* `haMQTTac/switch/power/set`
* `haMQTTac/mode/set`
* `haMQTTac/temperature/set`
* `haMQTTac/fan/set`
* `haMQTTac/vane/set`
* `haMQTTac/vane_left/set`
* `haMQTTac/wide_vane/set`

### States

* `haMQTTac/mitsu_ir/switch/power/state`
* `haMQTTac/mitsu_ir/mode/state`
* `haMQTTac/mitsu_ir/temperature/state`
* `haMQTTac/mitsu_ir/fan/state`
* `haMQTTac/mitsu_ir/vane/state`
* `haMQTTac/mitsu_ir/vane_left/state`
* `haMQTTac/mitsu_ir/wide_vane/state`
* `haMQTTac/mitsu_ir/availability`

---

## Button Actions

* **Tap**: Toggles A/C power (on/off) and applies last known settings

---

## Scheduled Reboot

* The device reboots every **Sunday at 01:00** to ensure long-term stability

---

## Home Assistant Integration

This project publishes MQTT discovery payloads for climate, switch, and select entities. Entities will appear automatically once the ESP32 connects.

---

## License

MIT License

---

Happy automating! 🚀
