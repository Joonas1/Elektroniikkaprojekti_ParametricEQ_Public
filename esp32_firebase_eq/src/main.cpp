#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <Firebase_ESP_Client.h>
#include <ArduinoJson.h>
#include <addons/RTDBHelper.h>
#include <time.h>
#include <WiFiClientSecure.h>

extern "C" {
  #include "esp_wifi.h"
  #include "esp_system.h"
}

#include "shared.h"
extern "C" void startControlTask();

// Define the single shared instance here (not extern!)
EqShared gEqShared = { 0 };   // not volatile
McpFeedback gMcpFeedback = { 0 };  // MCP readback values

#define EEPROM_SIZE 512
#define MAX_BANDS   7

// Keep both sides in sync at compile time
static_assert(SHARED_MAX_BANDS == MAX_BANDS, "Band count mismatch between shared_eq.h and main.cpp");

WebServer server(80);

// 🔑 Use SEPARATE FirebaseData objects
FirebaseData streamDO;   // RTDB stream ONLY
FirebaseData opDO;       // normal reads/writes (heartbeats, etc.)

FirebaseAuth auth;
FirebaseConfig config;

// ====== Data structures ======
struct Band { String type; float freq; float gain; float Q; bool enabled; };
struct EQState {
  float  gain = 0.0f;
  bool   power = false;
  Band   bands[MAX_BANDS];
  String filename;
  int    version = 0;
} eqState;

// ====== Stored credentials ======
String ssid, password, firebaseUrl, firebaseSecret;

// ====== Debug logging control (used by streamCallback and loop) ======
unsigned long lastDebugLog    = 0;
bool          pendingDebugLog = false;
// ====== EEPROM helpers ======
void writeStringToEEPROM(int &addr, const String &data) {
  for (size_t i = 0; i < data.length() && addr < (EEPROM_SIZE - 1); i++)
    EEPROM.write(addr++, static_cast<uint8_t>(data[i]));
  if (addr < EEPROM_SIZE) EEPROM.write(addr++, '\0');
}
String readStringFromEEPROM(int &addr) {
  String out;
  while (addr < EEPROM_SIZE) {
    uint8_t b = EEPROM.read(addr++);
    if (b == 0) break;
    out += static_cast<char>(b);
  }
  return out;
}

// ====== Captive config page ======
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>ESP32 Wi-Fi Setup</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body {font-family:Arial,sans-serif;background:#f4f4f4;padding:20px;color:#333;}
      form {background:#fff;max-width:420px;margin:30px auto;padding:20px;
            border-radius:10px;box-shadow:0 2px 8px rgba(0,0,0,0.2);}
      input[type=text],input[type=password]{
            display:block;width:calc(100% - 20px);margin:10px auto;
            padding:10px;border:1px solid #ccc;border-radius:5px;}
      input[type=submit],button{
            display:block;width:calc(100% - 20px);margin:10px auto;
            padding:10px;border:none;border-radius:5px;color:white;cursor:pointer;}
      input[type=submit]{background-color:#007bff;}
      input[type=submit]:hover{background-color:#0056b3;}
      button.clear{background-color:#dc3545;}
      button.clear:hover{background-color:#b52a37;}
    </style>
  </head>
  <body>
    <h2 style="text-align:center;">ESP32 Setup</h2>
    <form action="/save" method="post">
      <label>Wi-Fi SSID:</label>
      <input type="text" name="ssid" required>
      <label>Wi-Fi Password:</label>
      <input type="password" name="password" required>
      <label>Firebase URL:</label>
      <input type="text" name="firebaseUrl" required>
      <label>Legacy Database Secret:</label>
      <input type="text" name="firebaseSecret" required>
      <input type="submit" value="Save & Reboot">
    </form>
    <form action="/clear" method="post">
      <button class="clear">Clear Configuration</button>
    </form>
  </body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleSave() {
  ssid           = server.arg("ssid");
  password       = server.arg("password");
  firebaseUrl    = server.arg("firebaseUrl");
  firebaseSecret = server.arg("firebaseSecret");

  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  writeStringToEEPROM(addr, ssid);
  writeStringToEEPROM(addr, password);
  writeStringToEEPROM(addr, firebaseUrl);
  writeStringToEEPROM(addr, firebaseSecret);
  EEPROM.commit();
  EEPROM.end();

  server.send(200, "text/html", "<h3>Saved! Rebooting...</h3>");
  delay(1500);
  ESP.restart();
}

void handleClear() {
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < EEPROM_SIZE; i++) EEPROM.write(i, 0);
  EEPROM.commit();
  EEPROM.end();

  server.send(200, "text/html", "<h3>Configuration cleared! Rebooting...</h3>");
  delay(1500);
  ESP.restart();
}

bool readCredentials() {
  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  ssid           = readStringFromEEPROM(addr);
  password       = readStringFromEEPROM(addr);
  firebaseUrl    = readStringFromEEPROM(addr);
  firebaseSecret = readStringFromEEPROM(addr);
  EEPROM.end();
  return ssid.length() && firebaseUrl.length();
}

// Forward declaration
static void logWiperValuesToFirebase();

// ====== PUSH eqState -> gEqShared (fixed-point, seq-lock) ======
static void pushEqStateToShared()
{
  eqSharedBeginWrite(gEqShared);

  // master
  gEqShared.masterGain10  = clampGain10((int32_t)lroundf(eqState.gain * 10.0f));
  gEqShared.masterEnabled = eqState.power;

  // bands
  for (uint8_t i = 0; i < MAX_BANDS; ++i) {
    Band &src = eqState.bands[i];
    BandShared b;
    b.freq  = clampFreq((uint32_t)lroundf(src.freq));
    b.gain10  = clampGain10((int32_t)lroundf(src.gain * 10.0f));
    b.q10     = clampQ10((uint32_t)lroundf(src.Q * 10.0f));
    b.enabled = src.enabled;
    gEqShared.bands[i] = b;
  }

  eqSharedEndWrite(gEqShared);
}

// ====== Full-state parser ======
void applyFullStateFromJson(FirebaseJson &json) {
  FirebaseJsonData v;
  if (json.get(v, "gain")     && v.success) eqState.gain = v.to<float>();
  if (json.get(v, "power")    && v.success) eqState.power = v.to<bool>();
  if (json.get(v, "filename") && v.success) eqState.filename = v.to<String>();
  if (json.get(v, "version")  && v.success) eqState.version = v.to<int>();

  FirebaseJsonData arrData;
  if (json.get(arrData, "bands") && arrData.success && arrData.typeNum == FirebaseJson::JSON_ARRAY) {
    FirebaseJsonArray arr;
    arr.setJsonArrayData(arrData.to<String>());
    for (size_t i = 0; i < arr.size() && i < MAX_BANDS; i++) {
      FirebaseJsonData bandData;
      arr.get(bandData, i);
      if (!bandData.success || bandData.typeNum != FirebaseJson::JSON_OBJECT) continue;
      FirebaseJson bandJson; bandJson.setJsonData(bandData.to<String>());
      FirebaseJsonData b;
      if (bandJson.get(b, "type")    && b.success) eqState.bands[i].type    = b.to<String>();
      if (bandJson.get(b, "freq")    && b.success) eqState.bands[i].freq    = b.to<float>();
      if (bandJson.get(b, "gain")    && b.success) eqState.bands[i].gain    = b.to<float>();
      if (bandJson.get(b, "Q")       && b.success) eqState.bands[i].Q       = b.to<float>();
      if (bandJson.get(b, "enabled") && b.success) eqState.bands[i].enabled = b.to<bool>();
    }
  }
}

// ====== Stream callback ======
void streamCallback(FirebaseStream data) {
  const String path = data.dataPath();
  const String type = data.dataType();

  // Full node on first connect
  if ((path == "" || path == "/") && type == "json") {
    FirebaseJson fullJson = data.jsonObject();
    applyFullStateFromJson(fullJson);
    pushEqStateToShared();   // <-- reflect whole snapshot
    pendingDebugLog = true;  // <-- will log after control task processes
    Serial.println("📡 Initial full sync received.");
    return;
  }

  auto getFloat  = [&]() -> float  { return data.floatData(); };
  auto getBool   = [&]() -> bool   { return data.boolData();  };
  auto getInt    = [&]() -> int    { return data.intData();   };
  auto getString = [&]() -> String { return data.stringData(); };

  // Partial updates
  if (path.startsWith("/bands/")) {
    int idxStart = 7;
    int secondSlash = path.indexOf('/', idxStart);
    if (secondSlash > 0) {
      int bandIndex = path.substring(idxStart, secondSlash).toInt();
      String field  = path.substring(secondSlash + 1);
      if (bandIndex >= 0 && bandIndex < MAX_BANDS) {
        if      (field == "type")    eqState.bands[bandIndex].type    = getString();
        else if (field == "freq")    eqState.bands[bandIndex].freq    = getFloat();
        else if (field == "gain")    eqState.bands[bandIndex].gain    = getFloat();
        else if (field == "Q")       eqState.bands[bandIndex].Q       = getFloat();
        else if (field == "enabled") eqState.bands[bandIndex].enabled = getBool();

        pushEqStateToShared();  // <-- reflect band change
        pendingDebugLog = true;  // <-- will log after control task processes

        Serial.printf("🎚 Band %d %s = %s\n",
                      bandIndex, field.c_str(), data.stringData().c_str());
      }
    }
    return;
  }

  // Top-level fields
  if      (path == "/gain")     { eqState.gain     = getFloat();  Serial.printf("🎛 Master gain = %.2f dB\n", eqState.gain); }
  else if (path == "/power")    { eqState.power    = getBool();   Serial.printf("⚡ Power = %s\n", eqState.power ? "ON" : "OFF"); }
  else if (path == "/filename") { eqState.filename = getString(); Serial.printf("💾 File = %s\n", eqState.filename.c_str()); }
  else if (path == "/version")  { eqState.version  = getInt();    Serial.printf("📦 Version = %d\n", eqState.version); }
  else                          { Serial.printf("📡 Update %s = %s\n", path.c_str(), data.stringData().c_str()); }

  // Reflect any top-level change
  pushEqStateToShared();
  pendingDebugLog = true;  // <-- will log after control task processes
}

void streamTimeoutCallback(bool timeout) {
  if (timeout) Serial.println("⏳ Stream timeout, will attempt resume...");
}

// ====== Debug logging to Firebase ======
static void logWiperValuesToFirebase() {
  if (!Firebase.ready()) return;
  
  FirebaseJson debugJson;
  
  // Master
  int16_t masterGain10 = clampGain10((int32_t)lroundf(eqState.gain * 10.0f));
  uint8_t masterWiper = ((int32_t)masterGain10 + 150) * 255 / 300;
  debugJson.set("master/enabled", eqState.power);
  debugJson.set("master/gainDb", eqState.gain);
  debugJson.set("master/wiper", masterWiper);
  
  // Bands 0-3 (the ones used by hardware)
  for (int i = 0; i < 4; i++) {
    Band &b = eqState.bands[i];
    String prefix = "bands/" + String(i) + "/";
    
    // Calculate wiper values (same formulas as control.cpp)
    uint16_t freq = constrain((uint32_t)lroundf(b.freq), 20u, 20000u);
    float logFreq = log10f((float)freq);
    uint8_t freqWiper = (uint8_t)(((logFreq - log10f(20.0f)) / (log10f(20000.0f) - log10f(20.0f))) * 255.0f);
    
    uint16_t q10 = constrain((uint32_t)lroundf(b.Q * 10.0f), 1u, 1000u);
    uint8_t qWiper = (uint8_t)(((uint32_t)(q10 - 1) * 255) / 999);
    
    int16_t gain10 = constrain((int32_t)lroundf(b.gain * 10.0f), -150, 150);
    uint8_t gainWiper = (uint8_t)(((int32_t)gain10 + 150) * 255 / 300);
    
    debugJson.set(prefix + "enabled", b.enabled);
    debugJson.set(prefix + "freqHz", (int)freq);
    debugJson.set(prefix + "freqWiper", freqWiper);
    debugJson.set(prefix + "Q", b.Q);
    debugJson.set(prefix + "qWiper", qWiper);
    debugJson.set(prefix + "gainDb", b.gain);
    debugJson.set(prefix + "gainWiper", gainWiper);
  }
  
  // Add MCP feedback (actual values read from hardware)
  if (gMcpFeedback.valid) {
    for (int i = 0; i < 4; i++) {
      String prefix = "feedback/right/" + String(i) + "/";
      debugJson.set(prefix + "freq1", gMcpFeedback.rightFreq1[i]);
      debugJson.set(prefix + "freq2", gMcpFeedback.rightFreq2[i]);
      debugJson.set(prefix + "q1", gMcpFeedback.rightQ1[i]);
      debugJson.set(prefix + "q2", gMcpFeedback.rightQ2[i]);
      debugJson.set(prefix + "gain", gMcpFeedback.rightGain[i]);
      
      prefix = "feedback/left/" + String(i) + "/";
      debugJson.set(prefix + "freq1", gMcpFeedback.leftFreq1[i]);
      debugJson.set(prefix + "freq2", gMcpFeedback.leftFreq2[i]);
      debugJson.set(prefix + "q1", gMcpFeedback.leftQ1[i]);
      debugJson.set(prefix + "q2", gMcpFeedback.leftQ2[i]);
      debugJson.set(prefix + "gain", gMcpFeedback.leftGain[i]);
    }
    debugJson.set("feedback/right/master", gMcpFeedback.rightMaster);
    debugJson.set("feedback/left/master", gMcpFeedback.leftMaster);
    
    // I2C status info
    debugJson.set("i2c/tca0_ok", gMcpFeedback.tca0Ok);
    debugJson.set("i2c/tca1_ok", gMcpFeedback.tca1Ok);
    debugJson.set("i2c/write_success", gMcpFeedback.writeSuccessCount);
    debugJson.set("i2c/write_fail", gMcpFeedback.writeFailCount);
    debugJson.set("i2c/read_success", gMcpFeedback.readSuccessCount);
    debugJson.set("i2c/read_fail", gMcpFeedback.readFailCount);
    debugJson.set("i2c/initial_read_done", gMcpFeedback.initialReadDone);
    debugJson.set("i2c/last_update_ms", (int)gMcpFeedback.lastUpdateMs);
    
    // Per-potentiometer write success/fail tracking
    // Filter names: 0=Peak1, 1=Peak2, 2=Peak3, 3=LowShelf
    const char* filterNames[] = {"peak1", "peak2", "peak3", "lowShelf"};
    for (int i = 0; i < 4; i++) {
      String prefix = "potStats/right/" + String(filterNames[i]) + "/";
      debugJson.set(prefix + "freq1_ok", gMcpFeedback.rightFreq1WriteOk[i]);
      debugJson.set(prefix + "freq1_fail", gMcpFeedback.rightFreq1WriteFail[i]);
      debugJson.set(prefix + "freq2_ok", gMcpFeedback.rightFreq2WriteOk[i]);
      debugJson.set(prefix + "freq2_fail", gMcpFeedback.rightFreq2WriteFail[i]);
      debugJson.set(prefix + "q1_ok", gMcpFeedback.rightQ1WriteOk[i]);
      debugJson.set(prefix + "q1_fail", gMcpFeedback.rightQ1WriteFail[i]);
      debugJson.set(prefix + "q2_ok", gMcpFeedback.rightQ2WriteOk[i]);
      debugJson.set(prefix + "q2_fail", gMcpFeedback.rightQ2WriteFail[i]);
      debugJson.set(prefix + "gain_ok", gMcpFeedback.rightGainWriteOk[i]);
      debugJson.set(prefix + "gain_fail", gMcpFeedback.rightGainWriteFail[i]);
      
      prefix = "potStats/left/" + String(filterNames[i]) + "/";
      debugJson.set(prefix + "freq1_ok", gMcpFeedback.leftFreq1WriteOk[i]);
      debugJson.set(prefix + "freq1_fail", gMcpFeedback.leftFreq1WriteFail[i]);
      debugJson.set(prefix + "freq2_ok", gMcpFeedback.leftFreq2WriteOk[i]);
      debugJson.set(prefix + "freq2_fail", gMcpFeedback.leftFreq2WriteFail[i]);
      debugJson.set(prefix + "q1_ok", gMcpFeedback.leftQ1WriteOk[i]);
      debugJson.set(prefix + "q1_fail", gMcpFeedback.leftQ1WriteFail[i]);
      debugJson.set(prefix + "q2_ok", gMcpFeedback.leftQ2WriteOk[i]);
      debugJson.set(prefix + "q2_fail", gMcpFeedback.leftQ2WriteFail[i]);
      debugJson.set(prefix + "gain_ok", gMcpFeedback.leftGainWriteOk[i]);
      debugJson.set(prefix + "gain_fail", gMcpFeedback.leftGainWriteFail[i]);
    }
    debugJson.set("potStats/right/master_ok", gMcpFeedback.rightMasterWriteOk);
    debugJson.set("potStats/right/master_fail", gMcpFeedback.rightMasterWriteFail);
    debugJson.set("potStats/left/master_ok", gMcpFeedback.leftMasterWriteOk);
    debugJson.set("potStats/left/master_fail", gMcpFeedback.leftMasterWriteFail);
  }
  
  // Always log I2C error info (even if feedback not valid yet)
  debugJson.set("i2c/lastError", gMcpFeedback.lastError[0] ? gMcpFeedback.lastError : "none");
  debugJson.set("i2c/lastFailType", gMcpFeedback.lastFailType);
  debugJson.set("i2c/lastFailTca", gMcpFeedback.lastFailTca);
  debugJson.set("i2c/lastFailChannel", gMcpFeedback.lastFailChannel);
  debugJson.set("i2c/lastFailMcp", gMcpFeedback.lastFailMcp);
  debugJson.set("i2c/feedbackValid", gMcpFeedback.valid);
  
  // Last write debug info - shows last modified pot with sent vs read value
  char lastWriteMcpHex[8];
  snprintf(lastWriteMcpHex, sizeof(lastWriteMcpHex), "0x%02X", gMcpFeedback.lastWriteMcp);
  debugJson.set("lastWrite/name", gMcpFeedback.lastWriteName);
  debugJson.set("lastWrite/tca", gMcpFeedback.lastWriteTca);
  debugJson.set("lastWrite/channel", gMcpFeedback.lastWriteChannel);
  debugJson.set("lastWrite/mcp", lastWriteMcpHex);
  debugJson.set("lastWrite/valueSent", gMcpFeedback.lastWriteSentValue);
  debugJson.set("lastWrite/valueRead", gMcpFeedback.lastWriteReadValue);
  debugJson.set("lastWrite/success", gMcpFeedback.lastWriteSuccess);
  debugJson.set("lastWrite/verified", gMcpFeedback.lastWriteVerified);
  
  debugJson.set("timestamp", (int)millis());
  
  if (Firebase.RTDB.setJSON(&opDO, "/debug/wiperValues", &debugJson)) {
    Serial.println("📤 Wiper values logged to Firebase /debug/wiperValues");
  }
}

// ====== Firebase init ======
void initFirebase() {
  config.database_url = firebaseUrl.c_str();
  config.signer.tokens.legacy_token = firebaseSecret.c_str();

  // NOTE: TLS verification disabled for testing;
  // load a real root CA for production.
  config.cert.data = nullptr;

  Serial.println("🔒 Configuring Firebase TLS...");
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Tune buffers: larger for the stream connection
  streamDO.setBSSLBufferSize(16384, 16384);
  streamDO.setResponseSize(16384);
  streamDO.setCert(nullptr);

  // opDO can be smaller (but matching sizes is fine too)
  opDO.setBSSLBufferSize(8192, 8192);
  opDO.setResponseSize(8192);
  opDO.setCert(nullptr);

  delay(200);

  if (Firebase.ready()) {
    if (Firebase.RTDB.getInt(&opDO, "/state/version"))
      Serial.printf("🔥 Connected! Current version: %d\n", opDO.intData());
    else
      Serial.println("❌ RTDB test read failed: " + opDO.errorReason());
  }

  Serial.println("🔍 Starting Firebase stream on /state ...");
  if (!Firebase.RTDB.beginStream(&streamDO, "/state")) {
    Serial.println("❌ Stream begin failed!");
    Serial.println("Reason: " + streamDO.errorReason());
  } else {
    Firebase.RTDB.setStreamCallback(&streamDO, streamCallback, streamTimeoutCallback);
    Serial.println("✅ Stream started successfully!");
  }
}

// ====== Stream/heartbeat control ======
unsigned long lastHeartbeat   = 0;
unsigned long lastStreamPoll  = 0;
unsigned long lastStreamRetry = 0;
bool          streamHealthy   = true;

// exponential backoff
uint32_t retryBackoffMs = 2000;
const uint32_t retryBackoffMax = 30000;

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  WiFi.setAutoReconnect(true);
  Firebase.reconnectWiFi(true);
  delay(1000);

  if (readCredentials()) {
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid.c_str(), password.c_str());

    int timeout = 40;
    while (WiFi.status() != WL_CONNECTED && timeout-- > 0) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("✅ Wi-Fi connected!");
      Serial.println(WiFi.localIP());

      // Keep TLS alive
      WiFi.setSleep(false);
      esp_wifi_set_ps(WIFI_PS_NONE);

      // NTP
      configTime(7200, 0, "pool.ntp.org", "time.nist.gov");
      Serial.print("⏳ Syncing NTP time...");
      time_t now = time(nullptr);
      int retries = 0;
      while (now < 1000000000 && retries < 30) {
        delay(500); Serial.print("."); now = time(nullptr); retries++;
      }
      Serial.println();
      if (now > 1000000000) Serial.printf("✅ Time synced! Unix time: %ld\n", now);
      else                  Serial.println("⚠️ NTP sync failed, using millis() fallback.");

      // Web server
      server.on("/", handleRoot);
      server.on("/save",  HTTP_POST, handleSave);
      server.on("/clear", HTTP_POST, handleClear);
      server.begin();
      Serial.println("🌐 Web server started!");

      Serial.println("Connecting to Firebase...");
      initFirebase();
      Serial.println("[MAIN] starting ControlTask");
      startControlTask(); 
      return;
    }
  }

  // AP fallback
  Serial.println("Starting Access Point for setup...");
  WiFi.softAP("ESP32_Config", "12345678");
  Serial.println("AP IP: " + WiFi.softAPIP().toString());

  server.on("/", handleRoot);
  server.on("/save",  HTTP_POST, handleSave);
  server.on("/clear", HTTP_POST, handleClear);
  server.begin();
}

// ====== Loop ======
void loop() {
  server.handleClient();

  if (WiFi.status() != WL_CONNECTED) return;

  unsigned long now = millis();

  // 1) Poll stream when healthy (~2 Hz)
  if (now - lastStreamPoll > 500) {
    lastStreamPoll = now;
    if (streamHealthy) {
      if (!Firebase.RTDB.readStream(&streamDO)) {
        streamHealthy = false;
        lastStreamRetry = now;
        int code = streamDO.httpCode();
        String err = streamDO.errorReason();
        if (err.length() == 0) err = "(no reason)";
        Serial.printf("⚠️ Stream error: %s (http %d)\n", err.c_str(), code);
      }
    }
  }

  // 2) Reconnect with exponential backoff
  if (!streamHealthy && (now - lastStreamRetry > retryBackoffMs)) {
    lastStreamRetry = now;

    Serial.println("🔌 Reconnecting stream...");
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());

    Firebase.RTDB.endStream(&streamDO);
    delay(200);
    streamDO.clear();

    if (Firebase.RTDB.beginStream(&streamDO, "/state")) {
      streamDO.setBSSLBufferSize(16384, 16384);
      streamDO.setResponseSize(16384);
      Firebase.RTDB.setStreamCallback(&streamDO, streamCallback, streamTimeoutCallback);
      Serial.println("🔁 Stream resumed");
      streamHealthy = true;
      retryBackoffMs = 2000;
    } else {
      int code = streamDO.httpCode();
      String err = streamDO.errorReason();
      if (err.length() == 0) err = "(no reason)";
      Serial.printf("❌ Stream resume failed: %s (http %d)\n", err.c_str(), code);
      retryBackoffMs = min<uint32_t>(retryBackoffMs * 2, retryBackoffMax);
    }
  }

  // 3) Heartbeat every 5s using the **opDO** channel (not the stream one)
  if (Firebase.ready() && (now - lastHeartbeat > 5000)) {
    lastHeartbeat = now;
    time_t epoch; time(&epoch);
    int value = (epoch > 1000000000) ? static_cast<int>(epoch)
                                     : static_cast<int>(millis());
    Firebase.RTDB.setInt (&opDO, "/status/lastSeen", value);
    Firebase.RTDB.setBool(&opDO, "/status/esp32Online", true);
  }

  // 4) Log debug info to Firebase after a change (with delay for MCP feedback)
  if (pendingDebugLog && (now - lastDebugLog > 500)) {
    lastDebugLog = now;
    pendingDebugLog = false;
    logWiperValuesToFirebase();
  }

  delay(2);
}
