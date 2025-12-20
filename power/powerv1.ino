/****************************************************
 * ESP32 Solar Power + Cleaning/Protection Node (38-pin)
 *
 * Features:
 *  - WiFi (static IP) + ThingSpeak (read/write)
 *  - Local web dashboard (status + manual control)
 *  - OLED I2C display (0.91"/0.96")
 *  - 2x voltage sensors (25V modules, ratio ~5:1)
 *  - 2x current sensors (ACS712-type)
 *  - Power sharing relays (5W + 65W panels) + LED
 *  - Cleaning + protection (H-bridge 2 relays + pump relay)
 *  - Buttons (clean, protect toggle, emergency stop)
 *  - Alarm system (under-voltage, over-current)
 *  - Auto-clean: low power but high lux from weather node
 *  - ESP-NOW receive from Weather node (rain, dust, lux)
 *  - HTTP push to Fog ESP32 (192.168.0.20 /updatePower)
 *
 * Relay logic: ACTIVE-LOW (LOW = ON, HIGH = OFF)
 ****************************************************/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <math.h>

// =========================
//  WIFI + NETWORK CONFIG
// =========================

const char* WIFI_SSID     = "RAVAN";
const char* WIFI_PASSWORD = "54726426";

// Static IP for this node (power + cleaning)
IPAddress local_IP(192, 168, 0, 12);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(1, 1, 1, 1);
IPAddress dns2(1, 0, 0, 1);

// Fog ESP32 (for /updatePower)
IPAddress fog_IP(192, 168, 0, 20);

// UDP (optional debug / future use)
const uint16_t UDP_PORT_LISTEN = 5006;

// =========================
//  THINGSPEAK CONFIG
// =========================

const char* TS_WRITE_API_KEY = "2RZPCMIQFV34K9BS";
const char* TS_READ_API_KEY  = "KK28II1TGG2KSWDO";
const long  CHANNEL_ID       = 2995079;
/*
 * ThingSpeak fields:
 *  field1: V12  (V)
 *  field2: I12  (A)
 *  field3: V6   (V)
 *  field4: I6   (A)
 *  field5: sharingEnabled (0/1)
 *  field6: charging12 (0/1)
 *  field7: alarmCode (bit field)
 *          bit0: UV12
 *          bit1: OC12
 *          bit2: UV6
 *          bit3: OC6
 *          bit4: AUTO_CLEAN_TRIGGERED (low power in high lux)
 *  field8: remoteCommand
 *          0 = ignore
 *          1 = force sharing OFF (manual)
 *          2 = force sharing ON  (manual)
 *          3 = AUTO mode (sharing)
 */

// =========================
//  ESP-NOW (WEATHER NODE)
// =========================

uint8_t WEATHER_MAC[6] = {0xFC, 0xB4, 0x67, 0xF6, 0x12, 0x48}; // for reference (rx only)

typedef struct __attribute__((packed)) {
  char     src;       // 'W'
  uint8_t  rainFlag;  // 0/1
  uint8_t  cleanFlag; // 0/1 (weather request)
  uint8_t  dustHigh;  // 0/1
  uint16_t lux;
  uint16_t dust;
} WtoPeerPacket;

struct WeatherFlags {
  bool        rainFlag   = false;
  bool        cleanFlag  = false;
  bool        dustHigh   = false;
  uint16_t    lux        = 0;
  uint16_t    dust       = 0;
  unsigned long lastUpdateMs = 0;
  bool        valid      = false;
} weatherFlags;

// =========================
//  PINOUT (38-pin ESP32)
// =========================

// ----- ANALOG INPUTS (ADC1 ONLY) -----
const int PIN_V12_SENSOR = 34;  // voltage sensor for 12V line
const int PIN_I12_SENSOR = 35;  // ACS712 current sensor 12V
const int PIN_V6_SENSOR  = 32;  // voltage sensor for 6V line
const int PIN_I6_SENSOR  = 33;  // ACS712 current sensor 6V

// ----- POWER SHARING RELAYS -----
const int PIN_RELAY_5W   = 14;  // 5W panel relay
const int PIN_RELAY_65W  = 27;  // 65W panel relay

// ----- CLEANING / PROTECTION RELAYS (H-BRIDGE + PUMP) -----
const int PIN_MOTOR_FWD  = 16;  // motor forward relay
const int PIN_MOTOR_REV  = 17;  // motor reverse relay
const int PIN_PUMP       = 23;  // pump relay

// ----- STATUS LEDs -----
const int PIN_LED_12V    = 26;  // charging LED for 12V batt
const int PIN_LED_6V     = 25;  // charging LED for 6V batt
const int PIN_LED_SHARE  = 4;   // sharing status LED

// ----- BUZZER -----
const int PIN_BUZZER     = 15;  // active buzzer

// ----- BUTTONS (INPUT_PULLUP) -----
const int PIN_BTN_CLEAN      = 18; // manual cleaning
const int PIN_BTN_PROTECT    = 19; // toggle protection
const int PIN_BTN_EMERGENCY  = 13; // emergency stop

// ----- OLED (I2C) -----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =========================
//  RELAY LOGIC (ACTIVE-LOW)
// =========================

// apnar request moto: relay on hobe LOW, off hobe HIGH
const int RELAY_ON_LEVEL  = LOW;
const int RELAY_OFF_LEVEL = HIGH;

// =========================
 //  ADC & SENSOR CALIBRATION
// =========================

const float ADC_REF_VOLT = 3.3f;
const int   ADC_MAX      = 4095;

// 25 V voltage sensor ≈ 5:1
float V12_DIV_RATIO = 5.0f;
float V6_DIV_RATIO  = 5.0f;

// ACS712-type current sensors (example values)
float I12_SENSITIVITY = 0.100f;  // V per A
float I6_SENSITIVITY  = 0.100f;
float I12_OFFSET      = 1.65f;   // V at 0 A
float I6_OFFSET       = 1.65f;

// Auto sharing thresholds
float V12_FULL_ON  = 13.8f;   // sharing ON if >=
float V12_FULL_OFF = 13.2f;   // sharing OFF if <=

// Charging detection min current
float I_CHG_MIN = 0.15f;

// Alarm thresholds
float V12_UNDERVOLT = 11.5f;
float V6_UNDERVOLT  = 5.5f;
float I12_OVERCURR  = 5.0f;
float I6_OVERCURR   = 1.5f;

// Auto-clean thresholds
const uint16_t LUX_GOOD_SUN        = 800;   // bhalo roder lux
const float    V12_DIRTY_THRESHOLD = 12.0f; // eto kom hole mone korbo dhulo ache
const unsigned long AUTOCLEAN_MIN_INTERVAL_MS = 60UL * 60UL * 1000UL; // 1 ghonta te ekbar max

// =========================
//  GLOBAL STATE
// =========================

WebServer server(80);
WiFiUDP   udp;

float V12 = 0, I12 = 0;
float V6  = 0, I6  = 0;

bool sharingEnabled = false;
bool charging12 = false;
bool charging6  = false;
bool autoModeSharing   = true;   // sharing auto ki na

uint8_t alarmCode = 0;
uint8_t lastAlarmCode = 0;

// Shield / cleaning / protection state
enum ShieldPos {
  SHIELD_POS_UNKNOWN = 0,
  SHIELD_POS_UP,
  SHIELD_POS_DOWN
};

ShieldPos     shieldPos          = SHIELD_POS_UP; // dhore nicchi boot e shild upore
bool          shieldMoving       = false;
bool          protectionActive   = false;   // shild niche rekhe protection on
bool          cleaningActive     = false;   // cleaning cholche kina
unsigned long lastAutoCleanMs    = 0;

// Shield timing (no limit switches)
const unsigned long SHIELD_DOWN_TIME_MS = 2800; // 2.8 sec niche jabe
const unsigned long SHIELD_UP_TIME_MS   = 3000; // 2.7 sec upore jabe

// Button debouncing (simple)
bool lastBtnCleanState     = HIGH;
bool lastBtnProtectState   = HIGH;
bool lastBtnEmergencyState = HIGH;

// Timers
unsigned long lastSensorMs     = 0;
unsigned long lastOLEDMs       = 0;
unsigned long lastTSUpdateMs   = 0;
unsigned long lastTSReadMs     = 0;
unsigned long lastFogUpdateMs  = 0;
unsigned long lastUDPprintMs   = 0;

unsigned long sensorIntervalMs    = 1000;
unsigned long oledIntervalMs      = 700;
unsigned long tsUpdateIntervalMs  = 20000;
unsigned long tsReadIntervalMs    = 15000;
unsigned long fogUpdateIntervalMs = 5000;
unsigned long udpPrintIntervalMs  = 5000;

// =========================
//  UTILITY FUNCTIONS
// =========================

float readVoltageRaw(int pin) {
  int raw = analogRead(pin);
  return (raw * ADC_REF_VOLT) / ADC_MAX;
}

float readVoltageDiv(int pin, float ratio) {
  return readVoltageRaw(pin) * ratio;
}

float readCurrentSensor(int pin, float offsetV, float sensVperA) {
  float v = readVoltageRaw(pin);
  float I = (v - offsetV) / sensVperA;
  return I;
}

void buzzerBeep(unsigned int durationMs = 120) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(durationMs);
  digitalWrite(PIN_BUZZER, LOW);
}

// =========================
//  POWER SHARING RELAYS
// =========================

void setPowerSharingRelays(bool enable) {
  if (enable) {
    Serial.println("[SHARE] panel sharing ON (5W detach -> 65W to 6V)");
    digitalWrite(PIN_RELAY_5W, RELAY_ON_LEVEL);
    delay(200);
    digitalWrite(PIN_RELAY_65W, RELAY_ON_LEVEL);
  } else {
    Serial.println("[SHARE] panel sharing OFF (back to normal)");
    digitalWrite(PIN_RELAY_65W, RELAY_OFF_LEVEL);
    delay(200);
    digitalWrite(PIN_RELAY_5W, RELAY_OFF_LEVEL);
  }
}

void setSharing(bool enable, const char* reason) {
  if (sharingEnabled == enable) {
    Serial.print("[SHARE] already ");
    Serial.println(enable ? "ON" : "OFF");
    return;
  }
  Serial.print("[SHARE] set to ");
  Serial.print(enable ? "ON" : "OFF");
  Serial.print(" | reason: ");
  Serial.println(reason);

  sharingEnabled = enable;
  setPowerSharingRelays(enable);
  digitalWrite(PIN_LED_SHARE, enable ? HIGH : LOW);
  buzzerBeep(80);
}

// =========================
//  MOTOR / PUMP HELPERS
// =========================

void motorStop() {
  // motor bondho (duita relay off -> HIGH)
  digitalWrite(PIN_MOTOR_FWD, RELAY_OFF_LEVEL);
  digitalWrite(PIN_MOTOR_REV, RELAY_OFF_LEVEL);
}

void motorDown() {
  // shild niche jabe
  Serial.println("[MOTOR] shild niche jacche (DOWN)");
  digitalWrite(PIN_MOTOR_FWD, RELAY_ON_LEVEL);
  digitalWrite(PIN_MOTOR_REV, RELAY_OFF_LEVEL);
}

void motorUp() {
  // shild upore jabe
  Serial.println("[MOTOR] shild upore jacche (UP)");
  digitalWrite(PIN_MOTOR_FWD, RELAY_OFF_LEVEL);
  digitalWrite(PIN_MOTOR_REV, RELAY_ON_LEVEL);
}

void pumpOn() {
  Serial.println("[PUMP] pump ON (cleaning water)");
  digitalWrite(PIN_PUMP, RELAY_ON_LEVEL);
}

void pumpOff() {
  Serial.println("[PUMP] pump OFF");
  digitalWrite(PIN_PUMP, RELAY_OFF_LEVEL);
}

// =========================
//  SHIELD STATE HELPERS
// =========================

const char* shieldPosName(ShieldPos p) {
  switch (p) {
    case SHIELD_POS_UP: return "UP";
    case SHIELD_POS_DOWN: return "DOWN";
    case SHIELD_POS_UNKNOWN:
    default: return "UNKNOWN";
  }
}

void shieldMoveDownIfNeeded(const char* reason) {
  if (shieldMoving) {
    Serial.println("[SHIELD] already moving, ignoring new DOWN cmd");
    return;
  }

  if (shieldPos == SHIELD_POS_DOWN) {
    Serial.println("[SHIELD] already DOWN, ignoring DOWN cmd");
    return;
  }

  Serial.print("[SHIELD] DOWN cmd | reason: ");
  Serial.println(reason);

  shieldMoving = true;
  motorDown();
  delay(SHIELD_DOWN_TIME_MS);   // 2.8 sec time based
  motorStop();

  shieldPos = SHIELD_POS_DOWN;
  shieldMoving = false;

  Serial.println("[SHIELD] now at: DOWN (time based)");
}

void shieldMoveUpIfNeeded(const char* reason) {
  if (shieldMoving) {
    Serial.println("[SHIELD] already moving, ignoring UP cmd");
    return;
  }

  if (shieldPos == SHIELD_POS_UP) {
    Serial.println("[SHIELD] already UP, ignoring UP cmd");
    return;
  }

  Serial.print("[SHIELD] UP cmd | reason: ");
  Serial.println(reason);

  shieldMoving = true;
  motorUp();
  delay(SHIELD_UP_TIME_MS);     // 2.7 sec time based
  motorStop();

  shieldPos = SHIELD_POS_UP;
  shieldMoving = false;

  Serial.println("[SHIELD] now at: UP (time based)");
}

// =========================
//  CLEANING CYCLE
// =========================

void runCleaningCycle(const char* reason) {
  if (cleaningActive) {
    Serial.println("[CLEAN] already running, ignoring new request");
    return;
  }
  if (protectionActive) {
    Serial.println("[CLEAN] protection active (shild niche), cleaning skip kora holo");
    return;
  }

  cleaningActive = true;
  Serial.print("[CLEAN] starting cleaning cycle | reason: ");
  Serial.println(reason);
  buzzerBeep(150);

  // age shild upore niye nei (safe starting)
  if (shieldPos != SHIELD_POS_UP) {
    shieldMoveUpIfNeeded("clean start - ensure UP");
  }

  // duibar: pump 3s + niche 2.8s + upore 2.7s
  for (int cycle = 0; cycle < 2; cycle++) {
    Serial.print("[CLEAN] cycle ");
    Serial.println(cycle + 1);

    // pump on 3 sec (shild upore thakbe)
    pumpOn();
    delay(3000);  // 3 sec
    pumpOff();

    // shild niche jabe 2.8 sec
    shieldMoveDownIfNeeded("clean cycle down");

    delay(300);   // choto pause

    // abar upore 2.7 sec
    shieldMoveUpIfNeeded("clean cycle up");
  }

  // seshe shild upore thakbe
  shieldPos = SHIELD_POS_UP;
  cleaningActive = false;
  Serial.println("[CLEAN] cleaning sesh, shild UP");
  buzzerBeep(100);
}

// =========================
//  PROTECTION LOGIC
// =========================

void setProtection(bool enable, const char* reason) {
  if (protectionActive == enable) {
    Serial.print("[PROTECT] already ");
    Serial.println(enable ? "ON" : "OFF");
    return;
  }

  protectionActive = enable;
  Serial.print("[PROTECT] set to ");
  Serial.print(enable ? "ON" : "OFF");
  Serial.print(" | reason: ");
  Serial.println(reason);

  if (enable) {
    // protection on -> shild niche jabe
    shieldMoveDownIfNeeded("protection ON");
  } else {
    // protection off -> brishti sesh / manual off -> shild upore jabe
    shieldMoveUpIfNeeded("protection OFF");
  }
}

// =========================
//  WIFI + THINGSPEAK
// =========================

void connectWiFi() {
  Serial.println("[WiFi] Configuring static IP...");
  if (!WiFi.config(local_IP, gateway, subnet, dns1, dns2)) {
    Serial.println("[WiFi] WiFi.config FAILED (continue anyway)");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 60) {
    delay(250);
    Serial.print(".");
    retries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] CONNECTED. IP: ");
    Serial.println(WiFi.localIP());
    buzzerBeep(180);
  } else {
    Serial.println("[WiFi] connection FAILED.");
  }
}

void sendToThingSpeak() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = "http://api.thingspeak.com/update?api_key=";
  url += TS_WRITE_API_KEY;
  url += "&field1=" + String(V12, 2);
  url += "&field2=" + String(I12, 2);
  url += "&field3=" + String(V6, 2);
  url += "&field4=" + String(I6, 2);
  url += "&field5=" + String(sharingEnabled ? 1 : 0);
  url += "&field6=" + String(charging12 ? 1 : 0);
  url += "&field7=" + String((int)alarmCode);

  Serial.print("[TS] update URL: ");
  Serial.println(url);

  http.begin(url);
  int httpCode = http.GET();
  if (httpCode > 0) {
    Serial.print("[TS] update HTTP code: ");
    Serial.println(httpCode);
  } else {
    Serial.print("[TS] update failed, code: ");
    Serial.println(httpCode);
  }
  http.end();
}

// Read remote command from field8 (sharing control)
void readThingSpeakCommand() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (CHANNEL_ID <= 0) return;

  HTTPClient http;
  String url = "http://api.thingspeak.com/channels/";
  url += String(CHANNEL_ID);
  url += "/fields/8/last.txt?api_key=";
  url += TS_READ_API_KEY;

  http.begin(url);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    int cmd = payload.toInt();
    Serial.print("[TS] Remote command (field8): ");
    Serial.println(cmd);

    if (cmd == 1) {          // force OFF
      autoModeSharing = false;
      setSharing(false, "TS force OFF");
    } else if (cmd == 2) {   // force ON
      autoModeSharing = false;
      setSharing(true, "TS force ON");
    } else if (cmd == 3) {   // auto mode
      autoModeSharing = true;
      Serial.println("[TS] sharing AUTO mode from TS");
    }
  } else {
    Serial.print("[TS] read cmd HTTP error: ");
    Serial.println(httpCode);
  }
  http.end();
}

// =========================
//  FOG UPDATE (HTTP PUSH)
// =========================

void sendToFog() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[FOG] WiFi not connected, skip");
    return;
  }

  WiFiClient client;
  const uint16_t fogPort = 80;

  Serial.print("[FOG] Connecting to ");
  Serial.print(fog_IP);
  Serial.println("...");

  if (!client.connect(fog_IP, fogPort)) {
    Serial.println("[FOG] Connection FAILED");
    return;
  }

  String url = "/updatePower?";
  url += "v12="      + String(V12, 2);
  url += "&i12="     + String(I12, 2);
  url += "&v6="      + String(V6, 2);
  url += "&i6="      + String(I6, 2);
  url += "&share="   + String(sharingEnabled ? 1 : 0);
  url += "&chg12="   + String(charging12 ? 1 : 0);
  url += "&chg6="    + String(charging6 ? 1 : 0);
  url += "&alarm="   + String((int)alarmCode);
  url += "&shield="  + String(shieldPosName(shieldPos));
  url += "&protect=" + String(protectionActive ? 1 : 0);
  url += "&clean="   + String(cleaningActive ? 1 : 0);

  String req =
    String("GET ") + url + " HTTP/1.1\r\n" +
    "Host: " + fog_IP.toString() + "\r\n" +
    "Connection: close\r\n\r\n";

  client.print(req);
  Serial.println("[FOG] Sent request:");
  Serial.println(req);

  unsigned long start = millis();
  while (client.connected() && millis() - start < 1000) {
    while (client.available()) client.read();
  }
  client.stop();
  Serial.println("[FOG] Done");
}

// =========================
//  OLED & WEB UI
// =========================

String alarmString() {
  if (alarmCode == 0) return "NONE";
  String s = "";
  if (alarmCode & 0x01) s += "UV12 ";
  if (alarmCode & 0x02) s += "OC12 ";
  if (alarmCode & 0x04) s += "UV6 ";
  if (alarmCode & 0x08) s += "OC6 ";
  if (alarmCode & 0x10) s += "AUTO_CLEAN ";
  return s;
}

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // line 1: WiFi
  if (WiFi.status() == WL_CONNECTED) {
    display.print("ip: ");
    display.println(WiFi.localIP());
  } else {
    display.println("wifi: off");
  }

  // line 2: 12V
  display.print("12v ");
  display.print(V12, 1);
  display.print("v ");
  display.print(I12, 1);
  display.println("a");

  // line 3: 6V
  display.print(" 6v ");
  display.print(V6, 1);
  display.print("v ");
  display.print(I6, 1);
  display.println("a");

  // line 4: sharing + mode
  display.print("share:");
  display.print(sharingEnabled ? "on " : "off");
  display.print(" mode:");
  display.println(autoModeSharing ? "auto" : "man");

  // line 5: shield + protect
  display.print("shild:");
  display.print(shieldPosName(shieldPos));
  display.print(" prot:");
  display.println(protectionActive ? "on" : "off");

  // line 6: alarm / rain flag
  display.print("alm:");
  if (alarmCode == 0) display.print("ok ");
  else {
    display.print("0x");
    display.print(alarmCode, HEX);
    display.print(" ");
  }
  if (weatherFlags.valid) {
    display.print("rn:");
    display.print(weatherFlags.rainFlag ? 1 : 0);
  } else {
    display.print("rn:-");
  }

  display.display();
}

String htmlPage() {
  String s;
  s += "<!doctype html><html><head><meta charset='utf-8'>";
  s += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<meta http-equiv='refresh' content='5'>";
  s += "<title>Solar Power + Cleaning Node</title>";
  s += "<style>";
  s += "body{font-family:Arial;background:#111;color:#eee;margin:0;padding:1rem;}";
  s += ".card{background:#222;padding:1rem;border-radius:8px;margin-bottom:1rem;}";
  s += "h1{font-size:1.4rem;margin:0 0 .5rem 0;} h2{font-size:1.1rem;margin:0 0 .5rem 0;}";
  s += "table{width:100%;border-collapse:collapse;}td{padding:4px;}";
  s += ".btn{display:inline-block;padding:8px 12px;margin:4px;border-radius:6px;text-decoration:none;color:#fff;background:#007bff;}";
  s += ".btn.red{background:#c0392b;} .btn.green{background:#27ae60;} .btn.orange{background:#f39c12;}";
  s += ".badge{padding:2px 6px;border-radius:4px;font-size:.8rem;}";
  s += ".ok{background:#27ae60;} .warn{background:#f39c12;} .err{background:#c0392b;}";
  s += "</style></head><body>";

  // header
  s += "<div class='card'><h1>Solar Power + Cleaning Node</h1>";
  s += "<p>IP: ";
  if (WiFi.status() == WL_CONNECTED) s += WiFi.localIP().toString();
  else s += "DISCONNECTED";
  s += "</p>";
  s += "<p>Share mode: ";
  s += autoModeSharing ? "<span class='badge ok'>AUTO</span>" : "<span class='badge warn'>MANUAL</span>";
  s += " &nbsp; Share: ";
  s += sharingEnabled ? "<span class='badge ok'>ON</span>" : "<span class='badge err'>OFF</span>";
  s += "</p>";
  s += "<p>Shield: <span class='badge'>";
  s += shieldPosName(shieldPos);
  s += "</span> &nbsp; Protection: ";
  s += protectionActive ? "<span class='badge err'>ON</span>" : "<span class='badge ok'>OFF</span>";
  s += " &nbsp; Cleaning: ";
  s += cleaningActive ? "<span class='badge warn'>RUN</span>" : "<span class='badge ok'>IDLE</span>";
  s += "</p>";
  s += "<p>Alarm: ";
  if (alarmCode == 0) s += "<span class='badge ok'>NONE</span>";
  else s += "<span class='badge err'>" + alarmString() + "</span>";
  s += "</p></div>";

  // measurements
  s += "<div class='card'><h2>Measurements</h2><table>";
  s += "<tr><td>12V Voltage</td><td>" + String(V12, 2) + " V</td></tr>";
  s += "<tr><td>12V Current</td><td>" + String(I12, 2) + " A</td></tr>";
  s += "<tr><td>6V Voltage</td><td>" + String(V6, 2) + " V</td></tr>";
  s += "<tr><td>6V Current</td><td>" + String(I6, 2) + " A</td></tr>";
  s += "<tr><td>Charging 12V</td><td>" + String(charging12 ? "YES" : "NO") + "</td></tr>";
  s += "<tr><td>Charging 6V</td><td>" + String(charging6 ? "YES" : "NO") + "</td></tr>";
  s += "</table></div>";

  // weather flags
  s += "<div class='card'><h2>Weather Flags (ESP-NOW)</h2>";
  if (!weatherFlags.valid) {
    s += "<p>No ESP-NOW packet from Weather node yet.</p>";
  } else {
    unsigned long age = millis() - weatherFlags.lastUpdateMs;
    s += "<table>";
    s += "<tr><td>Rain Flag</td><td>";
    s += weatherFlags.rainFlag ? "<span class='badge err'>RAIN</span>" : "<span class='badge ok'>NO</span>";
    s += "</td></tr>";
    s += "<tr><td>Dust High</td><td>";
    s += weatherFlags.dustHigh ? "<span class='badge warn'>HIGH</span>" : "<span class='badge ok'>OK</span>";
    s += "</td></tr>";
    s += "<tr><td>Lux</td><td>" + String(weatherFlags.lux) + " lx</td></tr>";
    s += "<tr><td>Dust ADC</td><td>" + String(weatherFlags.dust) + "</td></tr>";
    s += "<tr><td>Age</td><td>" + String(age / 1000.0, 1) + " s</td></tr>";
    s += "</table>";
  }
  s += "</div>";

  // controls
  s += "<div class='card'><h2>Controls</h2>";
  s += "<p>Sharing:</p><p>";
  s += "<a class='btn green' href='/shareOn'>Share ON</a>";
  s += "<a class='btn red' href='/shareOff'>Share OFF</a>";
  s += "</p><p>";
  s += "<a class='btn' href='/shareAuto'>Share AUTO</a>";
  s += "<a class='btn' href='/shareManual'>Share MANUAL</a>";
  s += "</p><hr>";
  s += "<p>Protection & Shield:</p><p>";
  s += "<a class='btn orange' href='/protectOn'>Protect ON (shield down)</a>";
  s += "<a class='btn' href='/protectOff'>Protect OFF (shield up)</a>";
  s += "</p><p>";
  s += "<a class='btn' href='/shieldUp'>Force shield UP</a>";
  s += "<a class='btn red' href='/shieldDown'>Force shield DOWN</a>";
  s += "</p><hr>";
  s += "<p>Cleaning:</p><p>";
  s += "<a class='btn green' href='/cleanOnce'>Run cleaning once</a>";
  s += "</p><hr>";
  s += "<p>Misc:</p><p>";
  s += "<a class='btn' href='/beep'>Beep</a>";
  s += "</p></div>";

  s += "</body></html>";
  return s;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

// =========================
//  ESP-NOW CALLBACK
// =========================

void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(WtoPeerPacket)) return;

  WtoPeerPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.src != 'W') return;

  weatherFlags.rainFlag   = pkt.rainFlag != 0;
  weatherFlags.cleanFlag  = pkt.cleanFlag != 0;
  weatherFlags.dustHigh   = pkt.dustHigh != 0;
  weatherFlags.lux        = pkt.lux;
  weatherFlags.dust       = pkt.dust;
  weatherFlags.lastUpdateMs = millis();
  weatherFlags.valid      = true;

  Serial.println("[ESPNOW] Weather packet received:");
  Serial.print("  rainFlag : "); Serial.println(weatherFlags.rainFlag);
  Serial.print("  cleanFlag: "); Serial.println(weatherFlags.cleanFlag);
  Serial.print("  dustHigh : "); Serial.println(weatherFlags.dustHigh);
  Serial.print("  lux      : "); Serial.println(weatherFlags.lux);
  Serial.print("  dust     : "); Serial.println(weatherFlags.dust);
}

// =========================
//  SETUP
// =========================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[BOOT] Power + Cleaning node starting...");

  // pin modes
  pinMode(PIN_RELAY_5W, OUTPUT);
  pinMode(PIN_RELAY_65W, OUTPUT);
  pinMode(PIN_MOTOR_FWD, OUTPUT);
  pinMode(PIN_MOTOR_REV, OUTPUT);
  pinMode(PIN_PUMP, OUTPUT);

  pinMode(PIN_LED_12V, OUTPUT);
  pinMode(PIN_LED_6V, OUTPUT);
  pinMode(PIN_LED_SHARE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_BTN_CLEAN, INPUT_PULLUP);
  pinMode(PIN_BTN_PROTECT, INPUT_PULLUP);
  pinMode(PIN_BTN_EMERGENCY, INPUT_PULLUP);

  // initial relay state (all off: HIGH, because active-LOW)
  digitalWrite(PIN_RELAY_5W, RELAY_OFF_LEVEL);
  digitalWrite(PIN_RELAY_65W, RELAY_OFF_LEVEL);
  motorStop();
  pumpOff();

  digitalWrite(PIN_LED_12V, LOW);
  digitalWrite(PIN_LED_6V, LOW);
  digitalWrite(PIN_LED_SHARE, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  analogReadResolution(12);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED] init failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Power+Clean Node");
    display.println("Booting...");
    display.display();
  }

  // shield initial state
  shieldPos = SHIELD_POS_UP;
  Serial.print("[BOOT] initial shieldPos: ");
  Serial.println(shieldPosName(shieldPos));

  // WIFI
  connectWiFi();

  // WEB SERVER ROUTES
  server.on("/", handleRoot);
  server.on("/shareOn", []() {
    autoModeSharing = false;
    setSharing(true, "web shareOn");
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/shareOff", []() {
    autoModeSharing = false;
    setSharing(false, "web shareOff");
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/shareAuto", []() {
    autoModeSharing = true;
    Serial.println("[WEB] sharing mode -> AUTO");
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/shareManual", []() {
    autoModeSharing = false;
    Serial.println("[WEB] sharing mode -> MANUAL");
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.on("/protectOn", []() {
    setProtection(true, "web protectOn");
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/protectOff", []() {
    setProtection(false, "web protectOff");
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/shieldUp", []() {
    shieldMoveUpIfNeeded("web force UP");
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/shieldDown", []() {
    shieldMoveDownIfNeeded("web force DOWN");
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/cleanOnce", []() {
    runCleaningCycle("web manual clean");
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/beep", []() {
    buzzerBeep(150);
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
  Serial.println("[WEB] HTTP server started");

  // UDP (for future debug, optional)
  udp.begin(UDP_PORT_LISTEN);
  Serial.print("[UDP] Listening on port ");
  Serial.println(UDP_PORT_LISTEN);

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed");
  } else {
    esp_now_register_recv_cb(onEspNowRecv);
    Serial.println("[ESPNOW] ready (receive only)");
  }

  unsigned long now = millis();
  lastSensorMs     = now;
  lastOLEDMs       = now;
  lastTSUpdateMs   = now;
  lastTSReadMs     = now;
  lastFogUpdateMs  = now;
  lastUDPprintMs   = now;
}

// =========================
//  LOOP
// =========================

void loop() {
  unsigned long now = millis();
  server.handleClient();

  // WiFi auto reconnect (simple)
  if (WiFi.status() != WL_CONNECTED && (now % 10000 < 50)) {
    connectWiFi();
  }

  // ---- SENSOR READ ----
  if (now - lastSensorMs >= sensorIntervalMs) {
    lastSensorMs = now;

    V12 = readVoltageDiv(PIN_V12_SENSOR, V12_DIV_RATIO);
    V6  = readVoltageDiv(PIN_V6_SENSOR,  V6_DIV_RATIO);
    I12 = readCurrentSensor(PIN_I12_SENSOR, I12_OFFSET, I12_SENSITIVITY);
    I6  = readCurrentSensor(PIN_I6_SENSOR,  I6_OFFSET,  I6_SENSITIVITY);

    // choto current noise filter
    if (fabs(I12) < 0.05f) I12 = 0;
    if (fabs(I6)  < 0.05f) I6  = 0;

    charging12 = (I12 > I_CHG_MIN);
    charging6  = (I6  > I_CHG_MIN);

    digitalWrite(PIN_LED_12V, charging12 ? HIGH : LOW);
    digitalWrite(PIN_LED_6V,  charging6  ? HIGH : LOW);

    // ---- SHARING AUTO LOGIC ----
    if (autoModeSharing) {
      if (!sharingEnabled && V12 >= V12_FULL_ON) {
        setSharing(true, "auto V12 high");
      } else if (sharingEnabled && V12 <= V12_FULL_OFF) {
        setSharing(false, "auto V12 low");
      }
    }

    // ---- ALARM EVALUATION ----
    alarmCode = 0;
    if (V12 > 1 && V12 < V12_UNDERVOLT)  alarmCode |= 0x01;
    if (I12 > I12_OVERCURR)              alarmCode |= 0x02;
    if (V6 > 1 && V6 < V6_UNDERVOLT)     alarmCode |= 0x04;
    if (I6 > I6_OVERCURR)                alarmCode |= 0x08;

    // print debug
    Serial.println("===== Power Node Readings =====");
    Serial.print("V12: "); Serial.print(V12); Serial.print(" V, I12: "); Serial.print(I12); Serial.println(" A");
    Serial.print("V6 : "); Serial.print(V6);  Serial.print(" V, I6 : "); Serial.print(I6);  Serial.println(" A");
    Serial.print("share: "); Serial.print(sharingEnabled ? "ON" : "OFF");
    Serial.print(" | mode: "); Serial.println(autoModeSharing ? "AUTO" : "MAN");
    Serial.print("shild: "); Serial.println(shieldPosName(shieldPos));
    if (weatherFlags.valid) {
      Serial.print("lux: "); Serial.print(weatherFlags.lux);
      Serial.print(" | dustHigh: "); Serial.println(weatherFlags.dustHigh);
    }
    Serial.print("alarmCode: 0x"); Serial.println(alarmCode, HEX);
    Serial.println("================================");

    // new alarm beep
    if (alarmCode != 0 && alarmCode != lastAlarmCode) {
      buzzerBeep(250);
    }
    lastAlarmCode = alarmCode;
  }

  // ---- AUTO PROTECTION from WEATHER (RAIN) ----
  if (weatherFlags.valid) {
    static bool lastRainFlag = false;
    if (weatherFlags.rainFlag && !lastRainFlag) {
      setProtection(true, "auto rainFlag ON");
    } else if (!weatherFlags.rainFlag && lastRainFlag) {
      setProtection(false, "auto rainFlag OFF");
    }
    lastRainFlag = weatherFlags.rainFlag;
  }

  // ---- AUTO CLEAN if HIGH LUX but LOW V12 ----
  if (weatherFlags.valid && !cleaningActive && !protectionActive) {
    bool enoughSun = (weatherFlags.lux >= LUX_GOOD_SUN);
    bool lowV12    = (V12 > 1 && V12 < V12_DIRTY_THRESHOLD);

    if (enoughSun && lowV12) {
      if (now - lastAutoCleanMs > AUTOCLEAN_MIN_INTERVAL_MS) {
        alarmCode |= 0x10;  // mark as auto clean
        runCleaningCycle("auto low power in good sun");
        lastAutoCleanMs = now;
      }
    }
  }

  // ---- BUTTON HANDLING (simple edge detect) ----
  bool btnClean   = digitalRead(PIN_BTN_CLEAN);
  bool btnProtect = digitalRead(PIN_BTN_PROTECT);
  bool btnEmerg   = digitalRead(PIN_BTN_EMERGENCY);

  if (btnClean == LOW && lastBtnCleanState == HIGH) {
    Serial.println("[BTN] clean button pressed -> manual cleaning");
    runCleaningCycle("button clean");
  }
  if (btnProtect == LOW && lastBtnProtectState == HIGH) {
    // toggle protection
    setProtection(!protectionActive, "button toggle protect");
  }
  if (btnEmerg == LOW && lastBtnEmergencyState == HIGH) {
    Serial.println("[BTN] emergency button pressed -> motorStop + pumpOff");
    motorStop();
    pumpOff();
    cleaningActive = false;
  }

  lastBtnCleanState   = btnClean;
  lastBtnProtectState = btnProtect;
  lastBtnEmergencyState = btnEmerg;

  // ---- OLED ----
  if (now - lastOLEDMs >= oledIntervalMs) {
    lastOLEDMs = now;
    updateOLED();
  }

  // ---- ThingSpeak write ----
  if (now - lastTSUpdateMs >= tsUpdateIntervalMs) {
    lastTSUpdateMs = now;
    sendToThingSpeak();
  }

  // ---- ThingSpeak read (remote command) ----
  if (now - lastTSReadMs >= tsReadIntervalMs) {
    lastTSReadMs = now;
    readThingSpeakCommand();
  }

  // ---- Push to Fog (HTTP) ----
  if (now - lastFogUpdateMs >= fogUpdateIntervalMs) {
    lastFogUpdateMs = now;
    sendToFog();
  }

  // ---- UDP debug (optional) ----
  if (now - lastUDPprintMs >= udpPrintIntervalMs) {
    lastUDPprintMs = now;
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      Serial.print("[UDP] packet received (");
      Serial.print(packetSize);
      Serial.println(" bytes)");
      char buf[256];
      int len = udp.read(buf, sizeof(buf) - 1);
      if (len > 0) {
        buf[len] = '\0';
        Serial.print("[UDP] From ");
        Serial.print(udp.remoteIP());
        Serial.print(":");
        Serial.println(udp.remotePort());
        Serial.print("[UDP] Payload: ");
        Serial.println(buf);
      }
    }
  }
}
