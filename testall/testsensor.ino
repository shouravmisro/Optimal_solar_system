// ESP32 Modular Sensors Serial, OLED, Web, Motor/Pump Control

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <BH1750.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ====== WiFi Credentials =======
const char* ssid = "Srv 🔥";
const char* password = "54726426";
WebServer server(80);

// ThingSpeak setup
const char* THINGSPEAK_API_KEY = "2RZPCMIQFV34K9BS";
const char* THINGSPEAK_URL = "http://api.thingspeak.com/update";
const char* EXTRA_THINGSPEAK_API_KEY = "4GS4T5JKWHHUZ4Q5";
const char* EXTRA_THINGSPEAK_URL = "http://api.thingspeak.com/update";
const char* CONTROL_CHANNEL_ID = "2995192";
const char* CONTROL_READ_API_KEY = "EXWNR81IBUV8T18W";

// ====== OLED Setup =======
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ====== Pin Definitions =======
#define DHTPIN 13           // DHT11 digital data pin
#define DHTTYPE DHT11
#define LDR_PIN 34          // LDR analog output (A0-style)
#define RAIN_ANALOG_PIN 35  // Rain sensor analog pin
#define RAIN_DIGITAL_PIN 32 // Rain sensor digital pin
#define DUST_LED_PIN 27     // Set to your wiring!
#define DUST_ANALOG_PIN 39  // Set to your wiring!
#define RELAY1_PIN  26      // Motor direction 1 (IN1)
#define RELAY2_PIN  25      // Motor direction 2 (IN2)
#define RELAY_PUMP  14      // Pump relay
#define RELAY_TRIGGER LOW   // Most relay boards are active LOW

// ====== Sensor Objects =======
Adafruit_BME280 bme;        // I2C, default 0x76
Adafruit_INA219 ina219;     // I2C, default 0x40
BH1750 lightMeter;          // I2C, 0x23/0x5C
DHT dht(DHTPIN, DHTTYPE);

// ====== Timing & State =======
unsigned long lastPrint = 0;
const unsigned long interval = 2000;
unsigned long lastCheck = 0;
unsigned long lastThingSpeakUpdate = 0;
String serialInput = "";

float bmeTemp, bmeHum, bmePress, bmeAlt;
float inaBusV, inaShuntV, inaCurrent, inaPower;
float lux;
float dhtTemp, dhtHum;
int ldrRaw, rainA, rainD;
float dustDensity;

int relayON  = RELAY_TRIGGER;
int relayOFF = (RELAY_TRIGGER == LOW ? HIGH : LOW);

static int motorstate = 0; // 0 = CW, 1 = CCW
static int lastMotorCmd = 0; // for debouncing remote commands
static int lastPumpCmd = 0;

// ====== Sensor Functions =======
void readBME280() {
  bmeTemp = bme.readTemperature();
  bmeHum = bme.readHumidity();
  bmePress = bme.readPressure() / 100.0F;
  bmeAlt = bme.readAltitude(1013.25);
  Serial.print("[BME280] T: "); Serial.print(bmeTemp,2);
  Serial.print("C, H: "); Serial.print(bmeHum,2);
  Serial.print("%, P: "); Serial.print(bmePress,2);
  Serial.print("hPa, Alt: "); Serial.print(bmeAlt,1); Serial.println(" m");
}

void readINA219() {
  inaBusV = ina219.getBusVoltage_V();
  inaShuntV = ina219.getShuntVoltage_mV();
  inaCurrent = ina219.getCurrent_mA();
  inaPower = ina219.getPower_mW();
  Serial.print("[INA219] Bus V: "); Serial.print(inaBusV,3);
  Serial.print(" V, Shunt: "); Serial.print(inaShuntV,2);
  Serial.print(" mV, Current: "); Serial.print(inaCurrent,2);
  Serial.print(" mA, Power: "); Serial.print(inaPower,2); Serial.println(" mW");
}

void readBH1750() {
  lux = lightMeter.readLightLevel();
  Serial.print("[BH1750] Light: "); Serial.print(lux,1); Serial.println(" lx");
}

void readDHT11() {
  dhtTemp = dht.readTemperature();
  dhtHum = dht.readHumidity();
  Serial.print("[DHT11] T: "); Serial.print(dhtTemp,1); Serial.print("C, H: "); Serial.print(dhtHum,1); Serial.println("%");
}

void readLDR() {
  ldrRaw = analogRead(LDR_PIN);
  Serial.print("[LDR] ADC: "); Serial.println(ldrRaw);
}

void readRain() {
  rainA = analogRead(RAIN_ANALOG_PIN);
  rainD = digitalRead(RAIN_DIGITAL_PIN);
  Serial.print("[Rain] A: "); Serial.print(rainA); Serial.print(", D: "); Serial.println(rainD);
  // Optional: auto toggle
  if (!rainD) {
    if (motorstate == 0) {
      motorCW();
      motorstate = 1;
    }
  } else {
    if (motorstate == 1) {
      motorCCW();
      motorstate = 0;
    }
  }
}

float readDustSensor() {
  digitalWrite(DUST_LED_PIN, LOW);
  delayMicroseconds(280);
  int dustADC = analogRead(DUST_ANALOG_PIN);
  delayMicroseconds(40);
  digitalWrite(DUST_LED_PIN, HIGH);
  delayMicroseconds(9680);
  float voltage = dustADC * (3.3 / 4095.0);
  float density = (voltage - 0.6) / 0.5 * 100.0;
  if (density < 0) density = 0;
  Serial.print("[Dust] ADC: "); Serial.print(dustADC); Serial.print(", Density: "); Serial.print(density, 1); Serial.println(" ug/m3");
  return density;
}

// ====== Relay/Motor/Pump =======
void motorCW() {
  digitalWrite(RELAY1_PIN, relayOFF);
  digitalWrite(RELAY2_PIN, relayON);
  Serial.println("[Motor] CW (forward)");
  delay(5000);
  digitalWrite(RELAY1_PIN, relayOFF);
  digitalWrite(RELAY2_PIN, relayOFF);
  Serial.println("[Motor] Stopped");
  motorstate=1;
}
void motorCCW() {
  digitalWrite(RELAY1_PIN, relayON);
  digitalWrite(RELAY2_PIN, relayOFF);
  Serial.println("[Motor] CCW (reverse)");
  delay(5000);
  digitalWrite(RELAY1_PIN, relayOFF);
  digitalWrite(RELAY2_PIN, relayOFF);
  Serial.println("[Motor] Stopped");
}
void motorStop() {
  digitalWrite(RELAY1_PIN, relayOFF);
  digitalWrite(RELAY2_PIN, relayOFF);
  Serial.println("[Motor] Stopped");
}
void pumpOn()  { digitalWrite(RELAY_PUMP, LOW); Serial.println("[Pump] ON"); }
void pumpOff() { digitalWrite(RELAY_PUMP, HIGH); Serial.println("[Pump] OFF"); }

// ====== OLED Display Function =======
void displayOnOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("BME:"); display.print(bmeTemp,1); display.print("C ");
  display.print(bmeHum,0); display.print("% ");
  display.print(bmePress,1); display.println("hPa");
  display.print("Alt:"); display.print(bmeAlt,1); display.println("m");
  display.print("INA:"); display.print(inaBusV,2); display.print("V ");
  display.print(inaCurrent,1); display.print("mA ");
  display.print(inaPower,1); display.println("mW");
  display.print("Light:"); display.print(lux,0); display.println("lx");
  display.print("DHT:"); display.print(dhtTemp,1); display.print("C ");
  display.print(dhtHum,0); display.println("%");
  display.print("LDR:"); display.print(ldrRaw); display.print(" R:");
  display.print(rainA); display.print("/"); display.println(rainD);
  display.print("Dust:"); display.print(dustDensity,1); display.println("ug/m3");
  display.display();
}

// ====== Web Page Dashboard =======
String getSensorPage() {
  String html = "<html><head><meta http-equiv='refresh' content='3'><title>ESP32 Sensor Dashboard</title>";
  html += "<style>body{font-family:sans-serif;background:#eef;}ul{background:#fff;padding:1em 2em;border-radius:1em;max-width:400px;margin:1em auto 2em auto;box-shadow:0 3px 12px #0002;}button{font-size:1em;padding:0.6em 1.2em;margin:0.5em 0.6em;border-radius:7px;border:1px solid #bbb;background:#3460b4;color:#fff;cursor:pointer;}button:active{background:#173263;}</style>";
  html += "</head><body>";
  html += "<h2>ESP32 Sensor Dashboard</h2>";
  html += "<ul>";
  html += "<li><b>BME280:</b> " + String(bmeTemp,2) + " &deg;C, " + String(bmeHum,2) + "% RH, " + String(bmePress,2) + " hPa, Alt: " + String(bmeAlt,1) + " m</li>";
  html += "<li><b>INA219:</b> BusV: " + String(inaBusV,3) + " V, Shunt: " + String(inaShuntV,2) + " mV, Current: " + String(inaCurrent,2) + " mA, Power: " + String(inaPower,2) + " mW</li>";
  html += "<li><b>BH1750:</b> Light: " + String(lux,1) + " lx</li>";
  html += "<li><b>DHT11:</b> T: " + String(dhtTemp,1) + " &deg;C, H: " + String(dhtHum,1) + "%</li>";
  html += "<li><b>LDR:</b> ADC: " + String(ldrRaw) + "</li>";
  html += "<li><b>Rain:</b> Analog: " + String(rainA) + ", Digital: " + String(rainD) + "</li>";
  html += "<li><b>Dust:</b> " + String(dustDensity,1) + " ug/m3</li>";
  html += "</ul>";
  // ==== Add control buttons ====
  html += "<div style='text-align:center;'>";
  html += "<button onclick=\"sendCmd('toggle')\">Toggle Motor (CW/CCW)</button>";
  html += "<button onclick=\"sendCmd('stop')\">Stop Motor</button><br/>";
  html += "<button onclick=\"pumpCmd('on')\">Pump ON</button>";
  html += "<button onclick=\"pumpCmd('off')\">Pump OFF</button>";
  html += "</div>";
  html += "<script>";
  html += "function sendCmd(d){fetch('/motor',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'dir='+d}).then(r=>r.text()).then(t=>alert(t));}";
  html += "function pumpCmd(cmd){fetch('/pump',{method:'POST',headers:{'Content-Type':'text/plain'},body:cmd}).then(r=>r.text()).then(t=>alert(t));}";
  html += "</script>";
  html += "</body></html>";
  return html;
}



// ====== ThingSpeak Uploads =======
void sendToThingSpeak() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String extras = String(dhtTemp, 1) + "," + String(dhtHum, 1) + "," +
                    String(ldrRaw) + "," + String(rainA) + "," +
                    String(rainD) + "," + String(dustDensity, 1);
    String url = String(THINGSPEAK_URL) + "?api_key=" + THINGSPEAK_API_KEY
        + "&field1=" + String(bmeTemp, 2)
        + "&field2=" + String(bmeHum, 2)
        + "&field3=" + String(bmePress, 2)
        + "&field4=" + String(bmeAlt, 1)
        + "&field5=" + String(inaBusV, 3)
        + "&field6=" + String(inaCurrent, 2)
        + "&field7=" + String(lux, 1)
        + "&field8=" + extras;
    http.begin(url);
    int httpCode = http.GET();
    Serial.print("[ThingSpeak] HTTP Response: ");
    Serial.println(httpCode);
    http.end();
  }
}
void sendToExtraChannel() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(EXTRA_THINGSPEAK_URL) + "?api_key=" + EXTRA_THINGSPEAK_API_KEY
      + "&field1=" + String(dhtTemp, 1)
      + "&field2=" + String(dhtHum, 1)
      + "&field3=" + String(ldrRaw)
      + "&field4=" + String(rainA)
      + "&field5=" + String(rainD)
      + "&field6=" + String(dustDensity, 1);
    http.begin(url);
    int httpCode = http.GET();
    Serial.print("[ThingSpeak Extra] Response: ");
    Serial.println(httpCode);
    http.end();
  }
}

// ====== Remote Control Poll =======
void checkRemoteCommands() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "https://api.thingspeak.com/channels/2995192/feeds.json?api_key=EXWNR81IBUV8T18W&results=1";
    http.begin(url);
    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, payload);
      JsonObject feed = doc["feeds"][0];

      int motorCmd = feed["field1"] ? String(feed["field1"]).toInt() : 0;
      int pumpCmd  = feed["field2"] ? String(feed["field2"]).toInt() : 0;

      if (motorCmd == 1 && lastMotorCmd == 0) {
        if (motorstate == 0) { motorCW(); motorstate = 1; }
        else { motorCCW(); motorstate = 0; }
        lastMotorCmd = 1;
      } else if (motorCmd == 0) {
        lastMotorCmd = 0;
      }

      if (pumpCmd == 1 && lastPumpCmd == 0) {
        pumpOn(); delay(5000); pumpOff(); lastPumpCmd = 1;
      } else if (pumpCmd == 0) {
        lastPumpCmd = 0;
      }
    }
    http.end();
  }
}

// ====== WebServer Endpoints =======
void handleControlPage() {
  String html = "<html><head><title>ESP32 Control</title></head><body><h2>Motor/Pump Manual Control</h2>";
  html += "<form method='POST' action='/motor'><button name='dir' value='toggle'>Toggle Motor (CW/CCW)</button></form>";
  html += "<form method='POST' action='/pump'><button name='plain' value='on'>Pump ON</button></form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY_PUMP, OUTPUT);
  digitalWrite(RELAY1_PIN, relayOFF);
  digitalWrite(RELAY2_PIN, relayOFF);
  digitalWrite(RELAY_PUMP, HIGH); // Pump OFF initially (LOW=ON for most relays)

  Serial.begin(115200); delay(1000);
  Serial.println("\n[ESP32 Modular Sensor Test]");
  Wire.begin();
  pinMode(DUST_LED_PIN, OUTPUT); digitalWrite(DUST_LED_PIN, HIGH);
  bme.begin(0x76);
  ina219.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  dht.begin();
  pinMode(LDR_PIN, INPUT);
  pinMode(RAIN_ANALOG_PIN, INPUT);
  pinMode(RAIN_DIGITAL_PIN, INPUT);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { Serial.println("SSD1306 allocation failed"); while(1); }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("ESP32 Sensor Test");
  display.display();
  delay(1000);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected. IP address: "); Serial.println(WiFi.localIP());

  server.on("/", []() { server.send(200, "text/html", getSensorPage()); });
  server.on("/control", HTTP_GET, handleControlPage);

  // Add at the top (if not already present)
static int lastMotorDir = 0; // 0 = CW (next press), 1 = CCW (next press)

server.on("/motor", HTTP_POST, []() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!server.hasArg("dir")) {
    server.send(400, "text/plain", "Missing dir arg");
    return;
  }
  String dir = server.arg("dir");
  if (dir == "toggle") {
    if (lastMotorDir == 0) {
      motorCW();
      lastMotorDir = 1;
      server.send(200, "text/plain", "Motor CW");
    } else {
      motorCCW();
      lastMotorDir = 0;
      server.send(200, "text/plain", "Motor CCW");
    }
  } else if (dir == "cw") {
    motorCW();
    lastMotorDir = 1;
    server.send(200, "text/plain", "Motor CW");
  } else if (dir == "ccw") {
    motorCCW();
    lastMotorDir = 0;
    server.send(200, "text/plain", "Motor CCW");
  } else if (dir == "stop") {
    motorStop();
    server.send(200, "text/plain", "Motor Stopped");
  } else {
    server.send(400, "text/plain", "Invalid dir");
  }
});


  server.on("/pump", HTTP_POST, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    String cmd = server.hasArg("plain") ? server.arg("plain") : "";
    if (cmd == "on") {
      pumpOn(); server.send(200, "text/plain", "Pump ON");
    } else if (cmd == "off") {
      pumpOff(); server.send(200, "text/plain", "Pump OFF");
    } else {
      server.send(400, "text/plain", "Invalid pump command");
    }
  });

  server.begin();
}

// ====== MAIN LOOP =======

void loop() {
  // Sensor & OLED update
  if (millis() - lastPrint > interval) {
    lastPrint = millis();
    readBME280();
    readINA219();
    readBH1750();
    readDHT11();
    readLDR();
    readRain();
    dustDensity = readDustSensor();
    displayOnOLED();
    Serial.println("=======================");
  }

  // Send to ThingSpeak every 16 seconds
  if (millis() - lastThingSpeakUpdate > 16000) {
    lastThingSpeakUpdate = millis();
    sendToThingSpeak();
    sendToExtraChannel();
  }

  // Serial input (local manual control)
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      serialInput.trim();
      if (serialInput.length() > 0) {
        if (serialInput.equalsIgnoreCase("cw")) {
          motorCW();
          motorstate = 1;
        } else if (serialInput.equalsIgnoreCase("ccw")) {
          motorCCW();
          motorstate = 0;
        } else if (serialInput.equalsIgnoreCase("stop")) {
          motorStop();
        } else if (serialInput.equalsIgnoreCase("pump on")) {
          pumpOn();
        } else if (serialInput.equalsIgnoreCase("pump off")) {
          pumpOff();
        } else {
          Serial.println("Unknown command. Use: cw, ccw, stop, pump on, pump off");
        }
        serialInput = "";
      }
    } else {
      serialInput += inChar;
    }
  }

  // Poll remote ThingSpeak control channel (every 15 seconds)
  if (millis() - lastCheck > 15000) {
    lastCheck = millis();
    checkRemoteCommands();
  }

  // Local WebServer handler (always)
  server.handleClient();
}
