/****************************************************
 * WEATHER NODE (ESP32-W) - FINAL v2 (DO-based rain + Web UI)
 *
 * Features:
 *  - BH1750 (lux), BMP280 (temp, pressure, altitude)
 *  - Rain sensor:
 *      - DO digital pin = main rain detection
 *      - AO analog pin  = logging only (optional)
 *  - GP2Y1010 dust sensor (analog + LED)
 *  - 0.91"/0.96" I2C OLED (128x64)
 *  - Static IP WiFi (192.168.0.10)
 *  - Local HTTP monitoring page (auto-refresh 5s)
 *  - HTTP push to Fog ESP32 at 192.168.0.20 (/updateWeather)
 *  - UDP telemetry backup to Motor + Power nodes
 *  - ESP-NOW backup to Motor + Power nodes (flags + basic data)
 *  - ThingSpeak logging
 ****************************************************/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <esp_now.h>
#include <Wire.h>

// OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Light sensor
#include <BH1750.h>

// BMP280
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// -------------------- WIFI CONFIG --------------------
const char* WIFI_SSID     = "RAVAN";
const char* WIFI_PASSWORD = "54726426";

// Router: 192.168.0.1 / 255.255.255.0 / DHCP 100-200
// Weather node static IP:
IPAddress local_IP(192, 168, 0, 10);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(1, 1, 1, 1);
IPAddress dns2(1, 0, 0, 1);

// Fog IP (for HTTP /updateWeather)
IPAddress fog_IP(192, 168, 0, 20);

// Motor & Power nodes for UDP
IPAddress motor_IP(192, 168, 0, 11);
IPAddress power_IP(192, 168, 0, 12);

// ThingSpeak
const char* THINGSPEAK_HOST      = "api.thingspeak.com";
const char* THINGSPEAK_WRITE_KEY = "4GS4T5JKWHHUZ4Q5";  // weather channel

// -------------------- ESP-NOW PEER MACs --------------------
uint8_t MOTOR_MAC[6] = {0xB8, 0xD6, 0x1A, 0xAB, 0xE9, 0x30}; // Motor ESP32
uint8_t POWER_MAC[6] = {0xFC, 0xB4, 0x67, 0xF6, 0x00, 0x24}; // Power ESP32

// -------------------- PINS --------------------
// I2C: SDA=21, SCL=22
#define SDA_PIN 21
#define SCL_PIN 22

// Rain sensor
const int PIN_RAIN_AO = 34;   // rain sensor AO (analog)
const int PIN_RAIN_DO = 14;   // rain sensor DO (digital) -> MAIN rain detection

// Dust sensor
const int PIN_DUST_ADC = 35;  // GP2Y1010 analog output
const int PIN_DUST_LED = 4;   // GP2Y1010 IR LED control

// -------------------- OLED CONFIG --------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -------------------- SENSORS --------------------
BH1750 lightMeter;
Adafruit_BMP280 bmp;

#define SEALEVELPRESSURE_HPA 1013.25

// -------------------- NETWORK OBJECTS --------------------
WiFiUDP   udp;
WebServer server(80);

// -------------------- TIMINGS --------------------
const unsigned long SENSOR_INTERVAL_MS       = 1000;  // 1s
const unsigned long FOG_PUSH_INTERVAL_MS     = 5000;  // 5s
const unsigned long UDP_INTERVAL_MS          = 5000;  // 5s
const unsigned long THINGSPEAK_INTERVAL_MS   = 20000; // 20s
const unsigned long STATUS_PRINT_INTERVAL_MS = 5000;  // 5s
const unsigned long ESPNOW_INTERVAL_MS       = 5000;  // 5s

unsigned long lastSensorMs      = 0;
unsigned long lastFogMs         = 0;
unsigned long lastUDPms         = 0;
unsigned long lastThingSpeakMs  = 0;
unsigned long lastStatusMs      = 0;
unsigned long lastEspNowMs      = 0;

// -------------------- UDP PORTS --------------------
const uint16_t UDP_PORT_MOTOR = 5005;
const uint16_t UDP_PORT_POWER = 5006;

// -------------------- SENSOR VALUES --------------------
float lux             = 0;
float luxFilt         = 0;
float temperature     = 0;
float temperatureFilt = 0;
float pressure        = 0;
float humidity        = 0;  // BMP280 te nai, future BME280 jonne placeholder
float altitude        = 0;

int   rainAdc         = 0;  // AO theke
float rainFilt        = 0;
int   rainDigital     = 1;  // DO theke (1=dry, 0=wet)

int   dustAdc         = 0;
float dustFilt        = 0;

// Flags
bool rainFlag   = false;   // 1 hole shield niche jabe
bool dustHigh   = false;
bool cleanFlag  = false;

// Dust thresholds
const float LUX_DAY_MIN       = 200.0;
const int   DUST_HIGH_TH      = 400;  // tune based on ADC readings
const int   DUST_HYST         = 50;

unsigned long rainOnSince   = 0;
unsigned long rainOffSince  = 0;
unsigned long dustHighSince = 0;

// -------------------- ESP-NOW PACKET --------------------
typedef struct __attribute__((packed)) {
  char     src;       // 'W'
  uint8_t  rainFlag;  // 0/1
  uint8_t  cleanFlag; // 0/1
  uint8_t  dustHigh;  // 0/1
  uint16_t lux;
  uint16_t dust;
} WtoPeerPacket;

// -------------------- FORWARD DECLARATIONS --------------------
void readSensors();
void updateRainFlag();
void updateDustAndCleanFlags();
void updateOLED();
void sendToFog();
void sendUDP();
void sendToThingSpeak();
void sendEspNow();
String htmlPage();
void handleRoot();

// ============================================================
// INIT FUNCTIONS
// ============================================================
void setupWiFi() {
  Serial.println("[WiFi] Configuring static IP...");
  if (!WiFi.config(local_IP, gateway, subnet, dns1, dns2)) {
    Serial.println("[WiFi] WiFi.config FAILED (continuing anyway).");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 60) {
    delay(250);
    Serial.print(".");
    retry++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] CONNECTED!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] FAILED to connect (will still use ESP-NOW).");
  }
}

void initSensors() {
  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] SSD1306 init failed");
    while (1) delay(1000);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Weather Node boot...");
  display.display();

  // BH1750
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("[BH1750] init failed");
  } else {
    Serial.println("[BH1750] OK");
  }

  // BMP280
  bool bmpOK = false;
  Serial.println("[BMP280] Trying 0x76...");
  bmpOK = bmp.begin(0x76);
  if (!bmpOK) {
    Serial.println("[BMP280] Not found at 0x76, trying 0x77...");
    bmpOK = bmp.begin(0x77);
  }
  if (!bmpOK) {
    Serial.println("[BMP280] init failed (check wiring)");
  } else {
    Serial.println("[BMP280] OK");
  }

  // Rain sensor pins
  pinMode(PIN_RAIN_AO, INPUT);      // analog
  pinMode(PIN_RAIN_DO, INPUT);      // DO, module already has pull-up

  // Dust sensor pins
  pinMode(PIN_DUST_LED, OUTPUT);
  digitalWrite(PIN_DUST_LED, HIGH); // Off for GP2Y1010 (LED active low)
  pinMode(PIN_DUST_ADC, INPUT);
}

void initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] Init failed");
    return;
  }

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  memcpy(peerInfo.peer_addr, MOTOR_MAC, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESPNOW] Failed to add Motor peer");
  }

  memcpy(peerInfo.peer_addr, POWER_MAC, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESPNOW] Failed to add Power peer");
  }

  Serial.println("[ESPNOW] Initialized");
}

// ============================================================
// SENSOR + LOGIC
// ============================================================
void readSensors() {
  // Light
  lux = lightMeter.readLightLevel();

  // BMP280
  temperature = bmp.readTemperature();
  pressure    = bmp.readPressure() / 100.0F;
  altitude    = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  // humidity will remain 0 unless you add a real humidity sensor

  // Rain AO (logging only)
  rainAdc = analogRead(PIN_RAIN_AO);

  // Rain DO (main logic)
  rainDigital = digitalRead(PIN_RAIN_DO); // 1=dry, 0=wet (LED off hole 0)

  // Dust sensor sequence (from GP2Y1010 datasheet)
  digitalWrite(PIN_DUST_LED, LOW);      // turn LED on (active low)
  delayMicroseconds(280);
  dustAdc = analogRead(PIN_DUST_ADC);
  delayMicroseconds(40);
  digitalWrite(PIN_DUST_LED, HIGH);     // LED off
  delayMicroseconds(9680);

  // Low-pass filter (mainly for lux, temp, pressure, dust)
  static bool first = true;
  const float alpha = 0.3;

  if (first) {
    luxFilt         = lux;
    temperatureFilt = temperature;
    pressure        = pressure;
    rainFilt        = rainAdc;
    dustFilt        = dustAdc;
    first           = false;
  } else {
    luxFilt         = alpha * lux         + (1 - alpha) * luxFilt;
    temperatureFilt = alpha * temperature + (1 - alpha) * temperatureFilt;
    pressure        = alpha * pressure    + (1 - alpha) * pressure;
    rainFilt        = alpha * rainAdc     + (1 - alpha) * rainFilt;
    dustFilt        = alpha * dustAdc     + (1 - alpha) * dustFilt;
  }
}

// DO-based rain logic:
//  - DO == LOW  -> wet  -> rainFlag = 1 IMMEDIATELY (shild niche jabe)
//  - DO == HIGH -> dry -> 30 sec porjonto continuous dry thakle rainFlag = 0 (shild upore jabe)
void updateRainFlag() {
  bool isWet = (rainDigital == LOW); // DO low mane bhije gese (module LED off)
  unsigned long now = millis();

  if (isWet) {
    // ekbar bhijle sathe sathe rainFlag on
    if (!rainFlag) {
      rainFlag = true;
      rainOnSince = now;
      Serial.println("[RAIN-LOGIC] Rain ON (DO=0) -> rainFlag=1 (shild niche jabe)");
    }
  } else {
    // dry condition: rainFlag off hote 30 sec lagbe
    if (rainFlag && (now - rainOnSince > 30000)) { // 30s dry
      rainFlag = false;
      rainOffSince = now;
      Serial.println("[RAIN-LOGIC] Rain OFF (30s dry) -> rainFlag=0 (shild upore jabe)");
    }
  }

  // extra debug
  Serial.print("[RAIN-LOGIC] DO=");
  Serial.print(rainDigital);
  Serial.print(" isWet=");
  Serial.print(isWet ? 1 : 0);
  Serial.print(" rainFlag=");
  Serial.println(rainFlag ? 1 : 0);
}

void updateDustAndCleanFlags() {
  bool isDay      = luxFilt > LUX_DAY_MIN;
  bool isDustHigh = dustFilt > DUST_HIGH_TH;
  bool isDustLow  = dustFilt < (DUST_HIGH_TH - DUST_HYST);

  unsigned long now = millis();

  dustHigh = isDustHigh;

  if (!rainFlag && isDay && isDustHigh) {
    if (dustHighSince == 0) {
      dustHighSince = now;
    } else if (now - dustHighSince > 300000) { // 5 minutes
      cleanFlag = true;
    }
  } else {
    // reset timer if conditions not met
    dustHighSince = 0;
  }

  // If dust went back low, clear cleanFlag
  if (isDustLow) {
    cleanFlag = false;
  }
}

// ============================================================
// OLED
// ============================================================
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Weather Node (W)");

  display.setCursor(0, 10);
  display.print("IP:");
  if (WiFi.status() == WL_CONNECTED) {
    display.print(WiFi.localIP());
  } else {
    display.print("NoWiFi");
  }

  display.setCursor(0, 20);
  display.print("Lx:");
  display.print(luxFilt, 0);
  display.print(" T:");
  display.print(temperatureFilt, 1);
  display.print("C");

  display.setCursor(0, 30);
  display.print("P:");
  display.print(pressure, 0);
  display.print(" R_AO:");
  display.print(rainAdc);

  display.setCursor(0, 40);
  display.print("D:");
  display.print(dustAdc);
  display.print(" R_DO:");
  display.print(rainDigital);

  display.setCursor(0, 50);
  display.print("RainF:");
  display.print(rainFlag ? 1 : 0);
  display.print(" DustHi:");
  display.print(dustHigh ? 1 : 0);
  display.print(" Cln:");
  display.print(cleanFlag ? 1 : 0);

  display.display();
}

// ============================================================
// LOCAL WEB UI
// ============================================================
String htmlPage() {
  String s;
  s += "<!doctype html><html><head><meta charset='utf-8'>";
  s += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<meta http-equiv='refresh' content='5'>";
  s += "<title>Weather Node Monitor</title>";
  s += "<style>";
  s += "body{font-family:Arial;background:#111;color:#eee;margin:0;padding:1rem;}";
  s += ".card{background:#222;padding:1rem;border-radius:8px;margin-bottom:1rem;}";
  s += "h1{font-size:1.4rem;margin:0 0 .5rem 0;} h2{font-size:1.1rem;margin:0 0 .5rem 0;}";
  s += "table{width:100%;border-collapse:collapse;}td{padding:4px;}";
  s += ".badge{padding:2px 6px;border-radius:4px;font-size:.8rem;}";
  s += ".ok{background:#27ae60;} .warn{background:#f39c12;} .err{background:#c0392b;}";
  s += "</style></head><body>";

  s += "<div class='card'><h1>Weather Node (W)</h1>";
  s += "<p>IP: ";
  if (WiFi.status() == WL_CONNECTED) s += WiFi.localIP().toString();
  else s += "DISCONNECTED";
  s += "</p>";
  s += "<p>RainFlag: ";
  if (rainFlag) s += "<span class='badge err'>1 (shild niche jabe)</span>";
  else         s += "<span class='badge ok'>0 (shild upore thakbe)</span>";
  s += "</p>";
  s += "<p>CleanFlag: ";
  if (cleanFlag) s += "<span class='badge warn'>1 (cleaning request)</span>";
  else           s += "<span class='badge ok'>0</span>";
  s += "</p>";
  s += "</div>";

  s += "<div class='card'><h2>Measurements</h2><table>";
  s += "<tr><td>Lux</td><td>" + String(luxFilt, 1) + " lx</td></tr>";
  s += "<tr><td>Temperature</td><td>" + String(temperatureFilt, 1) + " &deg;C</td></tr>";
  s += "<tr><td>Pressure</td><td>" + String(pressure, 1) + " hPa</td></tr>";
  s += "<tr><td>Altitude</td><td>" + String(altitude, 1) + " m</td></tr>";
  s += "<tr><td>Rain AO</td><td>" + String(rainAdc) + "</td></tr>";
  s += "<tr><td>Rain DO</td><td>" + String(rainDigital) + (rainDigital ? " (dry)" : " (wet)") + "</td></tr>";
  s += "<tr><td>Dust ADC</td><td>" + String(dustAdc) + "</td></tr>";
  s += "<tr><td>DustHigh</td><td>";
  s += dustHigh ? "<span class='badge warn'>YES</span>" : "<span class='badge ok'>NO</span>";
  s += "</td></tr>";
  s += "</table></div>";

  s += "</body></html>";
  return s;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

// ============================================================
// NETWORK: FOG HTTP
// ============================================================
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

  String url = "/updateWeather?";
  url += "lux="       + String(luxFilt, 1);
  url += "&temp="     + String(temperatureFilt, 1);
  url += "&hum="      + String(humidity, 1);
  url += "&press="    + String(pressure, 1);
  url += "&rainAO="   + String(rainAdc);
  url += "&rainDO="   + String(rainDigital);
  url += "&dust="     + String(dustAdc);
  url += "&rainFlag=" + String(rainFlag ? 1 : 0);
  url += "&dustHigh=" + String(dustHigh ? 1 : 0);
  url += "&cleanFlag="+ String(cleanFlag ? 1 : 0);

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

// ============================================================
// NETWORK: UDP BACKUP
// ============================================================
void sendUDP() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[UDP] WiFi not connected, skip");
    return;
  }

  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"node\":\"W\",\"lux\":%.1f,\"temp\":%.1f,"
           "\"press\":%.1f,\"rainAO\":%d,\"rainDO\":%d,"
           "\"dust_adc\":%d,\"rain_flag\":%d,\"dust_high\":%d,"
           "\"clean_flag\":%d}",
           luxFilt, temperatureFilt, pressure,
           rainAdc, rainDigital,
           dustAdc,
           rainFlag ? 1 : 0, dustHigh ? 1 : 0, cleanFlag ? 1 : 0);

  Serial.println("[UDP] Sending telemetry to Motor & Power...");
  Serial.print("[UDP] Payload: ");
  Serial.println(payload);

  udp.beginPacket(motor_IP, UDP_PORT_MOTOR);
  udp.print(payload);
  udp.endPacket();

  udp.beginPacket(power_IP, UDP_PORT_POWER);
  udp.print(payload);
  udp.endPacket();

  Serial.println("[UDP] Telemetry sent.");
}

// ============================================================
// NETWORK: ThingSpeak
// ============================================================
void sendToThingSpeak() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[TS] WiFi not connected, skip");
    return;
  }

  WiFiClient tsClient;
  Serial.println("[TS] Connecting to api.thingspeak.com...");
  if (!tsClient.connect(THINGSPEAK_HOST, 80)) {
    Serial.println("[TS] Connection FAILED");
    return;
  }

  String url = "/update?api_key=";
  url += THINGSPEAK_WRITE_KEY;
  url += "&field1=" + String(luxFilt, 1);
  url += "&field2=" + String(temperatureFilt, 1);
  url += "&field3=" + String(pressure, 1);
  url += "&field4=" + String(rainAdc);           // AO
  url += "&field5=" + String(dustAdc);
  url += "&field6=" + String(rainFlag ? 1 : 0);  // rain flag (shild control)
  url += "&field7=" + String(dustHigh ? 1 : 0);
  url += "&field8=" + String(cleanFlag ? 1 : 0);

  String req = String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + THINGSPEAK_HOST + "\r\n" +
               "Connection: close\r\n\r\n";

  tsClient.print(req);
  Serial.println("[TS] Sent update");

  unsigned long start = millis();
  while (tsClient.connected() && millis() - start < 1500) {
    while (tsClient.available()) tsClient.read();
  }
  tsClient.stop();
}

// ============================================================
// NETWORK: ESP-NOW
// ============================================================
void sendEspNow() {
  WtoPeerPacket pkt;
  pkt.src      = 'W';
  pkt.rainFlag = rainFlag ? 1 : 0;
  pkt.cleanFlag= cleanFlag ? 1 : 0;
  pkt.dustHigh = dustHigh ? 1 : 0;
  pkt.lux      = (uint16_t)constrain((int)luxFilt, 0, 65535);
  pkt.dust     = (uint16_t)constrain((int)dustFilt, 0, 65535);

  esp_now_send(MOTOR_MAC, (uint8_t*)&pkt, sizeof(pkt));
  esp_now_send(POWER_MAC, (uint8_t*)&pkt, sizeof(pkt));

  Serial.println("[ESPNOW] Packet sent to Motor & Power");
}

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("=== WEATHER NODE (W) - FINAL v2 (DO rain + Web) ===");

  initSensors();
  setupWiFi();
  udp.begin(0);  // random local port
  initEspNow();

  // Local web server
  server.on("/", handleRoot);
  server.begin();
  Serial.println("[WEB] HTTP server started on port 80");

  unsigned long now = millis();
  lastSensorMs      = now;
  lastFogMs         = now;
  lastUDPms         = now;
  lastThingSpeakMs  = now;
  lastStatusMs      = now;
  lastEspNowMs      = now;
}

void loop() {
  unsigned long now = millis();

  // handle HTTP
  server.handleClient();

  // 1. Sensors & logic (1s)
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    readSensors();
    updateRainFlag();
    updateDustAndCleanFlags();
    updateOLED();
  }

  // 2. Push to Fog via HTTP (5s)
  if (now - lastFogMs >= FOG_PUSH_INTERVAL_MS) {
    lastFogMs = now;
    sendToFog();
  }

  // 3. UDP backup (5s)
  if (now - lastUDPms >= UDP_INTERVAL_MS) {
    lastUDPms = now;
    sendUDP();
  }

  // 4. ThingSpeak (20s)
  if (now - lastThingSpeakMs >= THINGSPEAK_INTERVAL_MS) {
    lastThingSpeakMs = now;
    sendToThingSpeak();
  }

  // 5. ESP-NOW (5s)
  if (now - lastEspNowMs >= ESPNOW_INTERVAL_MS) {
    lastEspNowMs = now;
    sendEspNow();
  }

  // 6. Status debug (5s)
  if (now - lastStatusMs >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusMs = now;

    Serial.println("===== Sensor Readings =====");
    Serial.print("Lux        : "); Serial.print(luxFilt);         Serial.println(" lx");
    Serial.print("Temp       : "); Serial.print(temperatureFilt); Serial.println(" C");
    Serial.print("Pressure   : "); Serial.print(pressure);        Serial.println(" hPa");
    Serial.print("Altitude   : "); Serial.print(altitude);        Serial.println(" m");
    Serial.print("Rain AO    : "); Serial.print(rainAdc);
    Serial.print("  Rain DO : "); Serial.print(rainDigital);
    Serial.print(" -> RAIN FLAG: "); Serial.println(rainFlag ? "ON (niche jabe)" : "OFF (upore thakbe)");
    Serial.print("Dust ADC   : "); Serial.print(dustAdc);
    Serial.print(" -> DUST HIGH: "); Serial.println(dustHigh ? "YES" : "NO");
    Serial.print("Clean Flag : "); Serial.println(cleanFlag ? "ON" : "OFF");
    Serial.print("WiFi       : ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Not Connected");
    Serial.print("Free heap  : "); Serial.println(ESP.getFreeHeap());
    Serial.println("===========================");
    Serial.println();
  }
}
