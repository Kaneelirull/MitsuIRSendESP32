// Define debug flag and number of IR send retries
// Enabling DEBUG will activate serial logging
#define DEBUG
#define IR_SEND_RETRIES 2  // Number of times to resend IR signal for reliability

// Include necessary libraries
#include <Button2.h>            // Button handling library
#include <Arduino.h>            // Core Arduino functions and types
#include <IRremoteESP8266.h>    // IR remote library for ESP8266/ESP32
#include <IRsend.h>             // IR send functionality
#include <ir_Mitsubishi.h>      // Mitsubishi-specific IR protocol definitions
#include <WiFi.h>               // WiFi connection library for ESP32
#include <time.h>

// MQTT and JSON libraries
#define MQTT_MAX_PACKET_SIZE 1024
#include <PubSubClient.h>       // MQTT client library
#include <ArduinoJson.h>        // JSON serialization/deserialization

// Over-the-Air update library
#include <ArduinoOTA.h>

// WiFi credentials
const char* WIFI_SSID     = "WIFINAME";
const char* WIFI_PASSWORD = "XXXXXXXX";

// Weekly reboot
time_t now;
struct tm timeinfo;
bool rebootedToday = false;


// MQTT server configuration
typedef uint16_t u16;
const char* MQTT_SERVER   = "192.168.0.165";
const u16  MQTT_PORT      = 1883;
const char* MQTT_USER     = "hamqtt";
const char* MQTT_PASS     = "4162";
const char* MQTT_CLIENT_ID= "mitsu_ir_controller";  // Unique client ID

// MQTT topic base and IR namespace
#define BASE_TOPIC        "haMQTTac"
#define IR_NS             BASE_TOPIC "/mitsu_ir"

// Command topics for Home Assistant integration
const char* TOPIC_POWER_SET   = BASE_TOPIC "/switch/power/set";
const char* TOPIC_MODE_SET   = BASE_TOPIC "/mode/set";
const char* TOPIC_TEMP_SET    = BASE_TOPIC "/temperature/set";
const char* TOPIC_FAN_SET     = BASE_TOPIC "/fan/set";
const char* TOPIC_VANE_SET    = BASE_TOPIC "/vane/set";
const char* TOPIC_VANE_L_SET  = BASE_TOPIC "/vane_left/set";
const char* TOPIC_WIDE_SET    = BASE_TOPIC "/wide_vane/set";

// State and availability topics under IR namespace
const char* IR_TOPIC_POWER_STATE   = IR_NS "/switch/power/state";
const char* IR_TOPIC_MODE_STATE   = IR_NS "/mode/state";
const char* IR_TOPIC_TEMP_STATE    = IR_NS "/temperature/state";
const char* IR_TOPIC_FAN_STATE     = IR_NS "/fan/state";
const char* IR_TOPIC_AVAIL         = IR_NS "/availability";
const char* IR_TOPIC_VANE_STATE    = IR_NS "/vane/state";
const char* IR_TOPIC_VANE_LEFT_STATE = IR_NS "/vane_left/state";
const char* IR_TOPIC_WIDE_STATE    = IR_NS "/wide_vane/state";

// GPIO pin assignments
const u16 kIrLed = 25;   // IR LED output pin
const int BTN_PIN = 39;   // Button input pin (with pull-up)


// Instantiate IR controller and button handler
IRMitsubishiAC ac(kIrLed);  // Mitsubishi AC IR emitter
Button2 button(BTN_PIN);     // Button2 instance for single button

// Variables to hold AC state
bool isOn = false;           // AC power state
int  temperature = 20;       // AC temperature setting
int  fanSpeed    = 4;        // Fan speed (1-6)
int  vanePos     = 0;        // Vertical vane position index
int  vaneLeftPos = 0;        // Left horizontal vane index
int  wideVanePos = 2;        // Wide vane (oscillation) index
String mode = "off";

// Networking objects
WiFiClient wifiClient;       // Generic WiFi client for MQTT
PubSubClient mqttClient(wifiClient);  // MQTT client
unsigned long lastMqttAttempt = 0;     // Timestamp of last MQTT connection attempt

// Forward declarations of helper functions
void connectWiFi();
bool connectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishState();
void applyAndSend();
void printState();

void setup() {
#ifdef DEBUG
  Serial.begin(115200);  // Initialize serial port for debug logging
  delay(200);            // Short delay for serial to initialize
#endif
  
  // Configure button pin as input with internal pull-up resistor
  pinMode(BTN_PIN, INPUT_PULLUP);

  // Initialize IR emitter hardware
  ac.begin();

  // Print initial state to serial (if DEBUG)
  printState();

  // Configure button tap (short press) handler using lambda
  button.setTapHandler([](Button2& btn) {
    if (!isOn) {
      // Turn AC on: set mode, temperature, fan speed, and vane positions
      ac.on(); ac.setMode(kMitsubishiAcCool);
      ac.setTemp(temperature);
      ac.setFan(fanSpeed);
      ac.setVane(vanePos);
      ac.setVaneLeft(vaneLeftPos);
      ac.setWideVane(wideVanePos);
      isOn = true;
    } else {
      // Turn AC off
      ac.off(); isOn = false;
    }
    // Send IR command and update state topics
    applyAndSend();
  });

  // Connect to WiFi network
  connectWiFi();

  // Setup NTP
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  while (time(nullptr) < 100000) {
    delay(500);
    Serial.println("⏳ Waiting for time sync...");
  }
  Serial.println("⏰ Time synchronized.");


  // Setup OTA updates
  ArduinoOTA.setHostname("ESP32_AC_Controller");
  ArduinoOTA.begin();

  // Configure MQTT server and callback
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // Allow immediate first connection
  lastMqttAttempt = millis() - 5000;
}

void loop() {

  //Check if WIFI has disconnected
  tryReconnectWiFi();

  //DO LOOP
  button.loop();             // Poll button state
  ArduinoOTA.handle();       // Handle OTA events

  // Attempt MQTT reconnect periodically if disconnected
  if (!mqttClient.connected() && millis() - lastMqttAttempt > 5000) {
    lastMqttAttempt = millis();
    connectMQTT();
  }
  // Keep MQTT client alive and process incoming messages
  mqttClient.loop();

  time(&now);
localtime_r(&now, &timeinfo);

  // Check if it's Sunday (0 = Sunday) and 01:00:00 exactly
  if (timeinfo.tm_wday == 0 && timeinfo.tm_hour == 1 && timeinfo.tm_min == 0 && timeinfo.tm_sec == 0 && !rebootedToday) {
    Serial.println("Scheduled reboot triggered.");
    rebootedToday = true;
    delay(1000);  // Allow time for MQTT/Serial flush if needed
    ESP.restart();
  }

  // Reset reboot flag after 02:00
  if (timeinfo.tm_hour == 2) {
    rebootedToday = false;
  }
}

void connectWiFi() {
  unsigned long backoff = 500;  // Initial retry delay (ms)
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Block until connected, with exponential backoff
  while (WiFi.status() != WL_CONNECTED) {
    delay(backoff + random(0, 500));  // Add jitter
    backoff = min(backoff * 2, (unsigned long)5000);  // Cap at 5s
  }
}

//Reconnect to WIFI if lost connection
void tryReconnectWiFi() {
  static unsigned long lastWifiAttempt = 0;

  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiAttempt > 10000) {
    Serial.println("WiFi disconnected, trying to reconnect...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiAttempt = millis();
  }
}

bool connectMQTT() {
  // Connect to MQTT broker with last will: offline message on IR_TOPIC_AVAIL
  if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                         IR_TOPIC_AVAIL, 1, true, "offline")) {
    // Publish online availability
    mqttClient.publish(IR_TOPIC_AVAIL, "online", true);

    // Register Home Assistant discovery for switch (power)
    StaticJsonDocument<1024> doc;
    char buf[1024];
    doc.clear();
    doc["name"] = "Mitsu IR Power";
    doc["command_topic"] = TOPIC_POWER_SET;
    doc["state_topic"]   = IR_TOPIC_POWER_STATE;
    doc["availability_topic"] = IR_TOPIC_AVAIL;
    doc["payload_on"]    = "COOL";
    doc["payload_off"]   = "OFF";
    doc["unique_id"]     = "mitsu_ir_power";
    doc["object_id"]     = "mitsu_ir_power";
    serializeJson(doc, buf);
    mqttClient.publish("homeassistant/switch/mitsu_ir_power/config", buf, true);

    // Register Home Assistant climate entity
doc.clear();
doc["name"] = "Mitsu IR AC";
doc["availability_topic"]          = IR_TOPIC_AVAIL;
doc["temperature_command_topic"]   = TOPIC_TEMP_SET;
doc["temperature_state_topic"]     = IR_TOPIC_TEMP_STATE;
doc["mode_command_topic"]          = TOPIC_MODE_SET;
doc["mode_state_topic"]            = IR_TOPIC_MODE_STATE;
JsonArray modes = doc.createNestedArray("modes");
modes.add("off");
modes.add("cool");
modes.add("heat");
modes.add("dry");
modes.add("fan_only");
modes.add("auto");

// ── FAN CONTROL ──
doc["fan_mode_command_topic"]      = TOPIC_FAN_SET;
doc["fan_mode_state_topic"]        = IR_TOPIC_FAN_STATE;
JsonArray fanModes = doc.createNestedArray("fan_modes");
fanModes.add("auto");
fanModes.add("low");
fanModes.add("medium");
fanModes.add("high");
fanModes.add("max");
fanModes.add("quiet");
fanModes.add("maxmax");
doc["fan_mode_state_template"]     =
  "{% if value == '0' %}auto"
  "{% elif value == '1' %}low"
  "{% elif value == '2' %}medium"
  "{% elif value == '3' %}high"
  "{% elif value == '4' %}max"
  "{% elif value == '5' %}maxmax"
  "{% elif value == '6' %}quiet"
  "{% endif %}";

doc["min_temp"]    = 16;
doc["max_temp"]    = 30;
doc["temp_step"]   = 1;
doc["unique_id"]   = "mitsu_ir_ac";
doc["object_id"]   = "mitsu_ir_ac";

serializeJson(doc, buf);
mqttClient.publish("homeassistant/climate/mitsu_ir_ac/config", buf, true);

    // Register select entities for vane positions
    StaticJsonDocument<1024> sel;
    char bufSel[1024];

    // Vertical vane
    sel.clear();
    sel["name"] = "Mitsu IR Vane";
    sel["command_topic"] = TOPIC_VANE_SET;
    sel["state_topic"]   = IR_TOPIC_VANE_STATE;
    JsonArray opts = sel.createNestedArray("options");
    opts.add("auto"); opts.add("highest"); opts.add("high"); opts.add("middle");
    opts.add("low"); opts.add("lowest"); opts.add("swing");
    sel["unique_id"] = "mitsu_ir_vane";
    sel["object_id"] = "mitsu_ir_vane";
    serializeJson(sel, bufSel);
    mqttClient.publish("homeassistant/select/mitsu_ir_vane/config", bufSel, true);

    // Left vane
    sel.clear();
    sel["name"] = "Mitsu IR Left Vane";
    sel["command_topic"] = TOPIC_VANE_L_SET;
    sel["state_topic"]   = IR_TOPIC_VANE_LEFT_STATE;
    opts = sel.createNestedArray("options");
    opts.add("auto"); opts.add("highest"); opts.add("high"); opts.add("middle");
    opts.add("low"); opts.add("lowest"); opts.add("swing");
    sel["unique_id"] = "mitsu_ir_vane_left";
    sel["object_id"] = "mitsu_ir_vane_left";
    serializeJson(sel, bufSel);
    mqttClient.publish("homeassistant/select/mitsu_ir_vane_left/config", bufSel, true);

    // Wide vane (oscillation range)
    sel.clear();
    sel["name"] = "Mitsu IR Wide Vane";
    sel["command_topic"] = TOPIC_WIDE_SET;
    sel["state_topic"]   = IR_TOPIC_WIDE_STATE;
    opts = sel.createNestedArray("options");
    opts.add("left_max"); opts.add("left"); opts.add("middle");
    opts.add("right"); opts.add("right_max"); opts.add("wide"); opts.add("auto");
    sel["unique_id"] = "mitsu_ir_wide_vane";
    sel["object_id"] = "mitsu_ir_wide_vane";
    serializeJson(sel, bufSel);
    mqttClient.publish("homeassistant/select/mitsu_ir_wide_vane/config", bufSel, true);

    // Subscribe to all command topics
    mqttClient.subscribe(TOPIC_POWER_SET);
    mqttClient.subscribe(TOPIC_MODE_SET);
    mqttClient.subscribe(TOPIC_TEMP_SET);
    mqttClient.subscribe(TOPIC_FAN_SET);
    mqttClient.subscribe(TOPIC_VANE_SET);
    mqttClient.subscribe(TOPIC_VANE_L_SET);
    mqttClient.subscribe(TOPIC_WIDE_SET);


    // Publish initial state of AC and topics
    publishState();
    return true;
  }
  return false;
}

// MQTT message handler: process incoming commands
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Build lowercase payload string
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.toLowerCase();
  msg.trim();

  // ───── POWER SET ─────
  if (String(topic) == TOPIC_POWER_SET) {
    if (msg == "off") {
      ac.off();
      isOn = false;
    } else {
      ac.on();
      isOn = true;
    }
    // Send IR with whatever the last mode, temp, fan & vane positions are:
    applyAndSend();
  }
  // ───── MODE SET ─────
  else if (String(topic) == TOPIC_MODE_SET) {
    mode = msg;  // store for state topic

    if (mode != "off") {
      ac.on();
      isOn = true;
      if      (mode == "cool")      ac.setMode(kMitsubishiAcCool);
      else if (mode == "heat")      ac.setMode(kMitsubishiAcHeat);
      else if (mode == "dry")       ac.setMode(kMitsubishiAcDry);
      else if (mode == "fan_only"   // HA’s fan-only mode
            || mode == "fan")      // optional backward-compat
                                    ac.setMode(kMitsubishiAcFan);
      else if (mode == "auto")      ac.setMode(kMitsubishiAcAuto);
      else                      ac.setMode(kMitsubishiAcCool);
      applyAndSend();
    } else {
      // explicit OFF → send IR “off” then state
      ac.off();
      isOn = false;
      applyAndSend();
    }
  }
  // ───── TEMPERATURE SET ─────
  else if (String(topic) == TOPIC_TEMP_SET) {
    temperature = msg.toInt();
    ac.setTemp(temperature);
    mqttClient.publish(IR_TOPIC_TEMP_STATE, String(temperature).c_str(), true);
    if (isOn) applyAndSend();
  }
    // ───── FAN MODE SET ─────
  else if (String(topic) == TOPIC_FAN_SET) {
    // msg is one of "auto","low","medium","high","max","quiet"
    int code;
    if      (msg == "auto")   code = 0;
    else if (msg == "low")    code = 1;
    else if (msg == "medium") code = 2;
    else if (msg == "high")   code = 3;
    else if (msg == "max")    code = 4;
    else if (msg == "quiet")  code = 6;
    else if (msg == "maxmax")    code = 5;
    else {
      Serial.print("⮕ Unmapped fan_mode ‘");
      Serial.print(msg);
      Serial.println("’, defaulting to auto (0)");
      code = 0;
    }

    // apply
    fanSpeed = code;  
    ac.setFan(code);
    mqttClient.publish(IR_TOPIC_FAN_STATE, String(code).c_str(), true);
    if (isOn) applyAndSend();
  }
  // ───── VANE, VANE_RIGHT, WIDE ─────
  else if (String(topic) == TOPIC_VANE_SET) {
    if      (msg == "auto") vanePos = 0;
    else if (msg == "highest") vanePos = 1;
    else if (msg == "high") vanePos = 2;
    else if (msg == "middle") vanePos = 3;
    else if (msg == "low") vanePos = 4;
    else if (msg == "lowest") vanePos = 5;
    else if (msg == "swing") vanePos = 7;
    ac.setVane(vanePos);
    mqttClient.publish(IR_TOPIC_VANE_STATE, msg.c_str(), true);
    if (isOn) applyAndSend();
    }

  // ───── VANE, VANE_LEFT, WIDE ─────
  else if (String(topic) == TOPIC_VANE_L_SET) {
    if      (msg == "auto") vaneLeftPos = 0;
    else if (msg == "highest") vaneLeftPos = 1;
    else if (msg == "high") vaneLeftPos = 2;
    else if (msg == "middle") vaneLeftPos = 3;
    else if (msg == "low") vaneLeftPos = 4;
    else if (msg == "lowest") vaneLeftPos = 5;
    else if (msg == "swing") vaneLeftPos = 7;
    ac.setVaneLeft(vaneLeftPos);
    mqttClient.publish(IR_TOPIC_VANE_LEFT_STATE, msg.c_str(), true);
    if (isOn) applyAndSend();
  }

  // ───── VANE, VANE_WIDE ─────
  else if (String(topic) == TOPIC_WIDE_SET) {
    if      (msg == "left_max") wideVanePos = 0;
    else if (msg == "left") wideVanePos = 1;
    else if (msg == "middle") wideVanePos = 2;
    else if (msg == "right") wideVanePos = 3;
    else if (msg == "right_max") wideVanePos = 4;
    else if (msg == "wide") wideVanePos = 5;
    else if (msg == "auto") wideVanePos = 7;
    ac.setWideVane(wideVanePos);
    mqttClient.publish(IR_TOPIC_WIDE_STATE, msg.c_str(), true);
    if (isOn) applyAndSend();
  }
}


// Publish current AC settings to MQTT state topics
void publishState() {
  mqttClient.publish(IR_TOPIC_AVAIL, "online", true);
  mqttClient.publish(IR_TOPIC_POWER_STATE, isOn ? "ON" : "OFF", true);
  mqttClient.publish(IR_TOPIC_MODE_STATE,  mode.c_str(), true);
  mqttClient.publish(IR_TOPIC_TEMP_STATE, String(temperature).c_str(), true);
  mqttClient.publish(IR_TOPIC_FAN_STATE, String(fanSpeed).c_str(), true);
  
  // Arrays of option strings for mapping indices back to human-readable state
  const char* vaneOptions[] = {"auto","highest","high","middle","low","lowest","","swing"};
  const char* wideOptions[] = {"left_max","left","middle","right","right_max","wide","","auto"};
  mqttClient.publish(IR_TOPIC_VANE_STATE, vaneOptions[vanePos], true);
  mqttClient.publish(IR_TOPIC_VANE_LEFT_STATE, vaneOptions[vaneLeftPos], true);
  mqttClient.publish(IR_TOPIC_WIDE_STATE, wideOptions[wideVanePos], true);
}

// Send IR signal and update state printouts
void applyAndSend() {
  for (int i = 0; i < IR_SEND_RETRIES; i++) {
    ac.begin();  // Re-initialize IR transmitter
    ac.send();   // Send the configured IR packet
    if (i == 0) delay(50);  // Short delay between retries
  }
  printState();     // Print state to Serial if DEBUG
  publishState();   // Publish new state over MQTT
}

// Print AC state to serial for debugging
void printState() {
#ifdef DEBUG
  Serial.println(F("Mitsubishi A/C current state:"));
  Serial.printf("  %s\n", ac.toString().c_str());
#endif
}
