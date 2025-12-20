/****************************************************
 * FOG NODE (ESP32-F) - CENTRAL DASHBOARD (UPDATED)
 *
 * Nodes:
 *   - Weather Node (W): 192.168.0.10  -> /updateWeather
 *   - Power+Cleaning Node (P): 192.168.0.12 -> /updatePower
 *
 * Features:
 *   - Single web dashboard:
 *       * Weather data
 *       * Power + cleaning + protection data
 *       * WiFi + ThingSpeak connectivity
 *       * Node update ages
 *       * Command history (fog UI actions)
 *   - Control buttons:
 *       * /ctrl/shareOn      -> P:/shareOn
 *       * /ctrl/shareOff     -> P:/shareOff
 *       * /ctrl/autoMode     -> P:/shareAuto
 *       * /ctrl/manualMode   -> P:/shareManual
 *       * /ctrl/protectOn    -> P:/protectOn
 *       * /ctrl/protectOff   -> P:/protectOff
 *       * /ctrl/shieldDown   -> P:/shieldDown
 *       * /ctrl/shieldUp     -> P:/shieldUp
 *       * /ctrl/cleanOnce    -> P:/cleanOnce
 *       * /ctrl/beep         -> P:/beep
 ****************************************************/

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>

// ---------- WIFI CONFIG ----------
const char* WIFI_SSID     = "RAVAN";
const char* WIFI_PASSWORD = "54726426";

// Fog static IP
IPAddress local_IP(192, 168, 0, 20);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(1, 1, 1, 1);
IPAddress dns2(1, 0, 0, 1);

// Node IPs
IPAddress weather_IP(192, 168, 0, 10);
IPAddress power_IP(192, 168, 0, 12);  // power + cleaning + protection node

// ---------- ThingSpeak INFO (connectivity check) ----------
const char* TS_HOST            = "api.thingspeak.com";
// power sharing channel
const char* TS_POWER_CH_ID     = "2995079";
const char* TS_POWER_READKEY   = "KK28II1TGG2KSWDO";
// weather channel
const char* TS_WEATHER_CH_ID   = "2995090";
const char* TS_WEATHER_READKEY = "OYP628CRYJNOS0LC";

// ---------- GLOBAL SERVER ----------
WebServer server(80);

// ---------- DATA STRUCTURES ----------
struct WeatherData {
  bool   valid = false;
  float  lux = 0;
  float  temp = 0;
  float  hum = 0;
  float  press = 0;
  int    rainAO = 0;
  int    rainDO = 1;
  int    dust = 0;
  bool   rainFlag = false;
  bool   dustHigh = false;
  bool   cleanFlag = false;
  unsigned long lastUpdateMs = 0;
};

struct PowerNodeData {
  bool   valid = false;
  float  V12 = 0;
  float  I12 = 0;
  float  V6  = 0;
  float  I6  = 0;
  bool   sharing = false;
  bool   chg12 = false;
  bool   chg6  = false;
  uint8_t alarmCode = 0;

  // cleaning/protection/shield info from /updatePower
  String shieldState;        // "UP", "DOWN", "UNKNOWN"
  bool   protectionActive = false;
  bool   cleaningActive   = false;

  unsigned long lastUpdateMs = 0;
};

struct TsStatus {
  bool powerOK   = false;
  bool weatherOK = false;
  int  powerHttpCode   = 0;
  int  weatherHttpCode = 0;
  unsigned long lastCheckMs = 0;
};

WeatherData   gWeather;
PowerNodeData gPower;
TsStatus      gTsStatus;

// ---------- COMMAND HISTORY ----------
const int CMD_HISTORY_MAX = 10;
struct CommandEvent {
  unsigned long t;
  String source;  // e.g. "Fog-UI"
  String text;    // e.g. "shareOn -> Power OK"
};
CommandEvent gHistory[CMD_HISTORY_MAX];
int gHistoryCount = 0;

// ========== HELPERS ==========

String formatAge(unsigned long lastMs) {
  if (lastMs == 0) return "no data";
  unsigned long now = millis();
  unsigned long diff = (now >= lastMs) ? now - lastMs : 0;
  unsigned long sec = diff / 1000;
  char buf[32];
  snprintf(buf, sizeof(buf), "%lus ago", sec);
  return String(buf);
}

void addHistory(const String& src, const String& txt) {
  if (gHistoryCount < CMD_HISTORY_MAX) {
    gHistory[gHistoryCount].t      = millis();
    gHistory[gHistoryCount].source = src;
    gHistory[gHistoryCount].text   = txt;
    gHistoryCount++;
  } else {
    for (int i = 1; i < CMD_HISTORY_MAX; i++) {
      gHistory[i - 1] = gHistory[i];
    }
    gHistory[CMD_HISTORY_MAX - 1].t      = millis();
    gHistory[CMD_HISTORY_MAX - 1].source = src;
    gHistory[CMD_HISTORY_MAX - 1].text   = txt;
  }
}

String htmlEscape(const String& s) {
  String out;
  out.reserve(s.length() + 10);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '&') out += "&amp;";
    else out += c;
  }
  return out;
}

// ---------- WIFI ----------

void connectWiFi() {
  Serial.println("[WiFi] configuring static IP...");
  if (!WiFi.config(local_IP, gateway, subnet, dns1, dns2)) {
    Serial.println("[WiFi] WiFi.config FAILED (continuing).");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WiFi] connecting to ");
  Serial.println(WIFI_SSID);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 60) {
    delay(250);
    Serial.print(".");
    retry++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] CONNECTED. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] FAILED to connect.");
  }
}

// ---------- THINGSPEAK CONNECTIVITY CHECK ----------

void checkThingSpeakOnce() {
  if (WiFi.status() != WL_CONNECTED) {
    gTsStatus.powerOK   = false;
    gTsStatus.weatherOK = false;
    gTsStatus.powerHttpCode   = -1;
    gTsStatus.weatherHttpCode = -1;
    gTsStatus.lastCheckMs = millis();
    return;
  }

  HTTPClient http;

  // power channel
  String urlP = String("http://") + TS_HOST +
                "/channels/" + TS_POWER_CH_ID +
                "/feeds/last.json?api_key=" + TS_POWER_READKEY;
  http.begin(urlP);
  int codeP = http.GET();
  gTsStatus.powerHttpCode = codeP;
  gTsStatus.powerOK = (codeP == 200);
  http.end();

  // weather channel
  String urlW = String("http://") + TS_HOST +
                "/channels/" + TS_WEATHER_CH_ID +
                "/feeds/last.json?api_key=" + TS_WEATHER_READKEY;
  http.begin(urlW);
  int codeW = http.GET();
  gTsStatus.weatherHttpCode = codeW;
  gTsStatus.weatherOK = (codeW == 200);
  http.end();

  gTsStatus.lastCheckMs = millis();

  Serial.print("[TS] power http=");   Serial.print(codeP);
  Serial.print(" weather http=");     Serial.println(codeW);
}

// ---------- FORWARD CONTROL TO POWER NODE ----------

bool sendSimpleGet(IPAddress ip, uint16_t port, const String& path) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClient client;
  Serial.print("[CTRL] connecting to ");
  Serial.print(ip);
  Serial.print(" ");
  Serial.print(path);
  Serial.println(" ...");

  if (!client.connect(ip, port)) {
    Serial.println("[CTRL] connect FAILED");
    return false;
  }

  String req =
    String("GET ") + path + " HTTP/1.1\r\n" +
    "Host: " + ip.toString() + "\r\n" +
    "Connection: close\r\n\r\n";

  client.print(req);

  unsigned long start = millis();
  while (client.connected() && millis() - start < 1000) {
    while (client.available()) client.read();
  }
  client.stop();
  Serial.println("[CTRL] done");
  return true;
}

// wrappers (all to power_IP)
void ctrlShareOn() {
  bool ok = sendSimpleGet(power_IP, 80, "/shareOn");
  addHistory("Fog-UI", ok ? "shareOn -> Power OK" : "shareOn -> Power FAIL");
}
void ctrlShareOff() {
  bool ok = sendSimpleGet(power_IP, 80, "/shareOff");
  addHistory("Fog-UI", ok ? "shareOff -> Power OK" : "shareOff -> Power FAIL");
}
void ctrlAutoMode() {
  bool ok = sendSimpleGet(power_IP, 80, "/shareAuto");
  addHistory("Fog-UI", ok ? "shareAuto -> Power OK" : "shareAuto -> Power FAIL");
}
void ctrlManualMode() {
  bool ok = sendSimpleGet(power_IP, 80, "/shareManual");
  addHistory("Fog-UI", ok ? "shareManual -> Power OK" : "shareManual -> Power FAIL");
}
void ctrlBeep() {
  bool ok = sendSimpleGet(power_IP, 80, "/beep");
  addHistory("Fog-UI", ok ? "beep -> Power OK" : "beep -> Power FAIL");
}
void ctrlProtectOn() {
  bool ok = sendSimpleGet(power_IP, 80, "/protectOn");
  addHistory("Fog-UI", ok ? "protectOn -> Power OK" : "protectOn -> Power FAIL");
}
void ctrlProtectOff() {
  bool ok = sendSimpleGet(power_IP, 80, "/protectOff");
  addHistory("Fog-UI", ok ? "protectOff -> Power OK" : "protectOff -> Power FAIL");
}
void ctrlShieldDown() {
  bool ok = sendSimpleGet(power_IP, 80, "/shieldDown");
  addHistory("Fog-UI", ok ? "shieldDown -> Power OK" : "shieldDown -> Power FAIL");
}
void ctrlShieldUp() {
  bool ok = sendSimpleGet(power_IP, 80, "/shieldUp");
  addHistory("Fog-UI", ok ? "shieldUp -> Power OK" : "shieldUp -> Power FAIL");
}
void ctrlCleanOnce() {
  bool ok = sendSimpleGet(power_IP, 80, "/cleanOnce");
  addHistory("Fog-UI", ok ? "cleanOnce -> Power OK" : "cleanOnce -> Power FAIL");
}

// ---------- HTML DASHBOARD ----------

String buildPage() {
  String h;
  h.reserve(9000);

  h += F("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
  h += F("<title>Fog Node Dashboard</title>");
  h += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  h += F("<style>"
         "body{font-family:'Segoe UI',Arial,sans-serif;background:#f6f8fa;color:#222;margin:0;}"
         "header{background:#173263;color:#fff;padding:1em 0.5em;text-align:center;"
         "box-shadow:0 2px 8px #0002;}"
         "h1{margin:0;font-size:1.8em;}"
         ".muted{color:#d0d6ea;font-size:0.9em;}"
         "main{max-width:1100px;margin:1em auto;padding:0 0.6em 2em 0.6em;}"
         ".cards{display:flex;flex-wrap:wrap;gap:1em;}"
         ".card{background:#fff;border-radius:12px;box-shadow:0 2px 12px #0002;"
         "padding:1em 1.1em;flex:1 1 310px;min-width:280px;}"
         ".card h2{margin:0 0 .6em 0;font-size:1.1em;color:#173263;}"
         "table{width:100%;border-collapse:collapse;font-size:0.92em;}"
         "td{padding:0.25em 0.2em;vertical-align:top;}"
         ".label{color:#555;}"
         ".value{font-weight:600;font-variant-numeric:tabular-nums;text-align:right;}"
         ".badge{display:inline-block;padding:0.1em 0.45em;border-radius:8px;font-size:0.78em;}"
         ".ok{background:#27ae60;color:#fff;}"
         ".warn{background:#f39c12;color:#fff;}"
         ".err{background:#c0392b;color:#fff;}"
         ".btn{display:inline-block;margin:0.15em 0.2em 0.15em 0; padding:0.35em 0.7em;"
         "border-radius:7px;background:#3460b4;color:#fff;text-decoration:none;font-size:0.9em;}"
         ".btn.red{background:#c0392b;}"
         ".btn.green{background:#27ae60;}"
         ".btn.orange{background:#f39c12;}"
         ".history{font-size:0.85em;color:#444;max-height:180px;overflow-y:auto;}"
         ".chip{display:inline-block;padding:0.1em 0.5em;border-radius:8px;background:#eee;font-size:0.8em;margin-bottom:0.2em;}"
         "@media(max-width:700px){.cards{flex-direction:column;}}"
         "</style></head><body>");

  h += F("<header><h1>Fog Node - Local Control Center</h1>");
  h += F("<div class='muted'>IP: ");
  if (WiFi.status() == WL_CONNECTED) {
    h += WiFi.localIP().toString();
    h += " | WiFi: CONNECTED";
  } else {
    h += "No IP | WiFi: DISCONNECTED";
  }
  h += F("</div></header><main><div class='cards'>");

  // ---- System / connectivity card ----
  h += F("<div class='card'><h2>System & Connectivity</h2><table>");
  h += F("<tr><td class='label'>WiFi</td><td class='value'>");
  if (WiFi.status() == WL_CONNECTED) h += F("<span class='badge ok'>CONNECTED</span>");
  else h += F("<span class='badge err'>DISCONNECTED</span>");
  h += F("</td></tr>");

  h += F("<tr><td class='label'>ThingSpeak Power</td><td class='value'>");
  if (gTsStatus.powerHttpCode == 0 && !gTsStatus.powerOK) {
    h += F("<span class='badge warn'>N/A</span>");
  } else if (gTsStatus.powerOK) {
    h += F("<span class='badge ok'>OK (200)</span>");
  } else {
    h += "<span class='badge err'>ERR(";
    h += String(gTsStatus.powerHttpCode);
    h += F(")</span>");
  }
  h += F("</td></tr>");

  h += F("<tr><td class='label'>ThingSpeak Weather</td><td class='value'>");
  if (gTsStatus.weatherHttpCode == 0 && !gTsStatus.weatherOK) {
    h += F("<span class='badge warn'>N/A</span>");
  } else if (gTsStatus.weatherOK) {
    h += F("<span class='badge ok'>OK (200)</span>");
  } else {
    h += "<span class='badge err'>ERR(";
    h += String(gTsStatus.weatherHttpCode);
    h += F(")</span>");
  }
  h += F("</td></tr>");

  h += F("<tr><td class='label'>TS last check</td><td class='value'>");
  h += formatAge(gTsStatus.lastCheckMs);
  h += F("</td></tr>");

  h += F("</table></div>");

  // ---- Weather card ----
  h += F("<div class='card'><h2>Weather Node (W)</h2><table>");
  h += F("<tr><td class='label'>Status</td><td class='value'>");
  if (gWeather.valid) {
    h += "<span class='badge ok'>OK</span> &nbsp; <span class='chip'>Updated ";
    h += formatAge(gWeather.lastUpdateMs);
    h += "</span>";
  } else {
    h += F("<span class='badge err'>NO DATA</span>");
  }
  h += F("</td></tr>");

  h += F("<tr><td class='label'>Lux</td><td class='value'>");
  h += String(gWeather.lux, 1) + " lx</td></tr>";
  h += F("<tr><td class='label'>Temp</td><td class='value'>");
  h += String(gWeather.temp, 1) + " °C</td></tr>";
  h += F("<tr><td class='label'>Humidity</td><td class='value'>");
  h += String(gWeather.hum, 1) + " %</td></tr>";
  h += F("<tr><td class='label'>Pressure</td><td class='value'>");
  h += String(gWeather.press, 1) + " hPa</td></tr>";

  h += F("<tr><td class='label'>Rain AO / DO</td><td class='value'>");
  h += String(gWeather.rainAO);
  h += " / ";
  h += String(gWeather.rainDO);
  h += F("</td></tr>");

  h += F("<tr><td class='label'>Dust ADC</td><td class='value'>");
  h += String(gWeather.dust);
  h += F("</td></tr>");

  h += F("<tr><td class='label'>Flags</td><td class='value'>");
  h += F("Rain:");
  h += gWeather.rainFlag ? "<span class='badge err'>ON</span> " : "<span class='badge ok'>OFF</span> ";
  h += F("Dust:");
  h += gWeather.dustHigh ? "<span class='badge warn'>HIGH</span> " : "<span class='badge ok'>OK</span> ";
  h += F("Clean:");
  h += gWeather.cleanFlag ? "<span class='badge orange'>REQ</span>" : "<span class='badge ok'>NO</span>";
  h += F("</td></tr>");

  h += F("</table></div>");

  // ---- Power + Cleaning card ----
  h += F("<div class='card'><h2>Power + Cleaning Node (P)</h2><table>");
  h += F("<tr><td class='label'>Status</td><td class='value'>");
  if (gPower.valid) {
    h += "<span class='badge ok'>OK</span> &nbsp; <span class='chip'>Updated ";
    h += formatAge(gPower.lastUpdateMs);
    h += "</span>";
  } else {
    h += F("<span class='badge err'>NO DATA</span>");
  }
  h += F("</td></tr>");

  h += F("<tr><td class='label'>12V Battery</td><td class='value'>");
  h += String(gPower.V12, 1) + " V / " + String(gPower.I12, 2) + " A</td></tr>";
  h += F("<tr><td class='label'>6V Battery</td><td class='value'>");
  h += String(gPower.V6, 1) + " V / " + String(gPower.I6, 2) + " A</td></tr>";

  h += F("<tr><td class='label'>Sharing</td><td class='value'>");
  h += gPower.sharing ? "<span class='badge ok'>ON</span>" : "<span class='badge err'>OFF</span>";
  h += F("</td></tr>");

  h += F("<tr><td class='label'>Charging</td><td class='value'>12V: ");
  h += gPower.chg12 ? "YES" : "NO";
  h += " &nbsp; 6V: ";
  h += gPower.chg6 ? "YES" : "NO";
  h += F("</td></tr>");

  h += F("<tr><td class='label'>Alarm</td><td class='value'>");
  if (!gPower.valid || gPower.alarmCode == 0) {
    h += F("<span class='badge ok'>NONE</span>");
  } else {
    h += "<span class='badge err'>0x";
    h += String(gPower.alarmCode, HEX);
    h += F("</span>");
  }
  h += F("</td></tr>");

  h += F("<tr><td class='label'>Shield</td><td class='value'>");
  h += htmlEscape(gPower.shieldState);
  h += F("</td></tr>");

  h += F("<tr><td class='label'>Protection</td><td class='value'>");
  h += gPower.protectionActive ? "<span class='badge err'>ON</span>" : "<span class='badge ok'>OFF</span>";
  h += F("</td></tr>");

  h += F("<tr><td class='label'>Cleaning</td><td class='value'>");
  h += gPower.cleaningActive ? "<span class='badge orange'>RUNNING</span>" : "<span class='badge ok'>IDLE</span>";
  h += F("</td></tr>");

  h += F("</table>");

  // control buttons
  h += F("<div style='margin-top:0.6em;'>");
  h += F("<div style='margin-bottom:0.3em;font-weight:600;'>Sharing</div>");
  h += F("<a class='btn green' href='/ctrl/shareOn'>Share ON</a>");
  h += F("<a class='btn red' href='/ctrl/shareOff'>Share OFF</a><br>");
  h += F("<a class='btn' href='/ctrl/autoMode'>AUTO mode</a>");
  h += F("<a class='btn' href='/ctrl/manualMode'>MANUAL mode</a>");
  h += F("</div>");

  h += F("<div style='margin-top:0.8em;'>");
  h += F("<div style='margin-bottom:0.3em;font-weight:600;'>Protection & Shield</div>");
  h += F("<a class='btn orange' href='/ctrl/protectOn'>Protect ON (shield down)</a>");
  h += F("<a class='btn' href='/ctrl/protectOff'>Protect OFF (shield up)</a><br>");
  h += F("<a class='btn' href='/ctrl/shieldUp'>Force Shield UP</a>");
  h += F("<a class='btn red' href='/ctrl/shieldDown'>Force Shield DOWN</a>");
  h += F("</div>");

  h += F("<div style='margin-top:0.8em;'>");
  h += F("<div style='margin-bottom:0.3em;font-weight:600;'>Cleaning</div>");
  h += F("<a class='btn green' href='/ctrl/cleanOnce'>Run Cleaning Once</a>");
  h += F("</div>");

  h += F("<div style='margin-top:0.8em;'>");
  h += F("<div style='margin-bottom:0.3em;font-weight:600;'>Misc</div>");
  h += F("<a class='btn' href='/ctrl/beep'>Beep</a>");
  h += F("</div>");

  h += F("</div>");  // end power card

  // ---- Command history card ----
  h += F("<div class='card'><h2>Command History (Fog)</h2><div class='history'>");
  if (gHistoryCount == 0) {
    h += F("<div class='muted'>No commands yet.</div>");
  } else {
    for (int i = gHistoryCount - 1; i >= 0; i--) {
      unsigned long ageMs = millis() - gHistory[i].t;
      unsigned long sec = ageMs / 1000;
      h += F("<div>");
      h += "<span class='chip'>";
      h += String(sec);
      h += "s ago</span> ";
      h += "<strong>";
      h += htmlEscape(gHistory[i].source);
      h += ":</strong> ";
      h += htmlEscape(gHistory[i].text);
      h += F("</div>");
    }
  }
  h += F("</div></div>");

  h += F("</div></main></body></html>");
  return h;
}

// ---------- HTTP HANDLERS ----------

void handleRoot() {
  String page = buildPage();
  server.send(200, "text/html", page);
}

// /updateWeather?lux=..&temp=..&hum=..&press=..&rainAO=..&rainDO=..&dust=..&rainFlag=..&dustHigh=..&cleanFlag=..
void handleUpdateWeather() {
  if (server.hasArg("lux"))      gWeather.lux      = server.arg("lux").toFloat();
  if (server.hasArg("temp"))     gWeather.temp     = server.arg("temp").toFloat();
  if (server.hasArg("hum"))      gWeather.hum      = server.arg("hum").toFloat();
  if (server.hasArg("press"))    gWeather.press    = server.arg("press").toFloat();
  if (server.hasArg("rainAO"))   gWeather.rainAO   = server.arg("rainAO").toInt();
  if (server.hasArg("rainDO"))   gWeather.rainDO   = server.arg("rainDO").toInt();
  if (server.hasArg("dust"))     gWeather.dust     = server.arg("dust").toInt();
  if (server.hasArg("rainFlag")) gWeather.rainFlag = server.arg("rainFlag").toInt() != 0;
  if (server.hasArg("dustHigh")) gWeather.dustHigh = server.arg("dustHigh").toInt() != 0;
  if (server.hasArg("cleanFlag"))gWeather.cleanFlag= server.arg("cleanFlag").toInt() != 0;

  gWeather.valid = true;
  gWeather.lastUpdateMs = millis();

  Serial.println("[FOG] /updateWeather received");
  server.send(200, "text/plain", "OK");
}

// /updatePower?v12=..&i12=..&v6=..&i6=..&share=..&chg12=..&chg6=..&alarm=..&shield=..&protect=..&clean=..
void handleUpdatePower() {
  if (server.hasArg("v12"))     gPower.V12     = server.arg("v12").toFloat();
  if (server.hasArg("i12"))     gPower.I12     = server.arg("i12").toFloat();
  if (server.hasArg("v6"))      gPower.V6      = server.arg("v6").toFloat();
  if (server.hasArg("i6"))      gPower.I6      = server.arg("i6").toFloat();
  if (server.hasArg("share"))   gPower.sharing = server.arg("share").toInt() != 0;
  if (server.hasArg("chg12"))   gPower.chg12   = server.arg("chg12").toInt() != 0;
  if (server.hasArg("chg6"))    gPower.chg6    = server.arg("chg6").toInt() != 0;
  if (server.hasArg("alarm"))   gPower.alarmCode = (uint8_t)server.arg("alarm").toInt();

  if (server.hasArg("shield"))  gPower.shieldState = server.arg("shield");
  if (server.hasArg("protect")) gPower.protectionActive = server.arg("protect").toInt() != 0;
  if (server.hasArg("clean"))   gPower.cleaningActive   = server.arg("clean").toInt() != 0;

  gPower.valid = true;
  gPower.lastUpdateMs = millis();

  Serial.println("[FOG] /updatePower received");
  server.send(200, "text/plain", "OK");
}

// control handlers -> redirect back to "/"
void handleCtrlShareOn()    { ctrlShareOn();    server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
void handleCtrlShareOff()   { ctrlShareOff();   server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
void handleCtrlAutoMode()   { ctrlAutoMode();   server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
void handleCtrlManualMode() { ctrlManualMode(); server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
void handleCtrlBeep()       { ctrlBeep();       server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }

void handleCtrlProtectOn()  { ctrlProtectOn();  server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
void handleCtrlProtectOff() { ctrlProtectOff(); server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
void handleCtrlShieldDown() { ctrlShieldDown(); server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
void handleCtrlShieldUp()   { ctrlShieldUp();   server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }
void handleCtrlCleanOnce()  { ctrlCleanOnce();  server.sendHeader("Location", "/", true); server.send(302, "text/plain", ""); }

// optional JSON status
void handleStatusJson() {
  String j = "{";
  j += "\"wifi\":"; j += (WiFi.status()==WL_CONNECTED ? "\"connected\"" : "\"disconnected\"");
  j += ",\"weather_valid\":"; j += (gWeather.valid ? "true" : "false");
  j += ",\"power_valid\":";   j += (gPower.valid   ? "true" : "false");
  j += "}";
  server.send(200, "application/json", j);
}

// ========== SETUP & LOOP ==========

unsigned long lastTsCheckMs = 0;
const unsigned long TS_CHECK_INTERVAL_MS = 60000; // 1 min

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("=== FOG NODE - CENTRAL DASHBOARD (UPDATED) ===");

  connectWiFi();

  // routes
  server.on("/", handleRoot);
  server.on("/status.json", handleStatusJson);

  server.on("/updateWeather", handleUpdateWeather);
  server.on("/updatePower",   handleUpdatePower);

  server.on("/ctrl/shareOn",    handleCtrlShareOn);
  server.on("/ctrl/shareOff",   handleCtrlShareOff);
  server.on("/ctrl/autoMode",   handleCtrlAutoMode);
  server.on("/ctrl/manualMode", handleCtrlManualMode);
  server.on("/ctrl/beep",       handleCtrlBeep);

  server.on("/ctrl/protectOn",  handleCtrlProtectOn);
  server.on("/ctrl/protectOff", handleCtrlProtectOff);
  server.on("/ctrl/shieldDown", handleCtrlShieldDown);
  server.on("/ctrl/shieldUp",   handleCtrlShieldUp);
  server.on("/ctrl/cleanOnce",  handleCtrlCleanOnce);

  server.begin();
  Serial.println("[WEB] HTTP server started");

  checkThingSpeakOnce();  // first check
}

void loop() {
  server.handleClient();

  unsigned long now = millis();

  // simple WiFi reconnect
  static unsigned long lastWifiCheck = 0;
  if (now - lastWifiCheck > 10000) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] lost, reconnecting...");
      connectWiFi();
    }
  }

  // periodic TS connectivity check
  if (now - lastTsCheckMs > TS_CHECK_INTERVAL_MS) {
    lastTsCheckMs = now;
    checkThingSpeakOnce();
  }
}
