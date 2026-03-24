#include <WiFi.h>
#include <WebServer.h>
#include <HX711.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/*
  ======================================================================
  MEGAJAVCHACK AUTOMATIC EGG SORTER
  DUAL GATE VERSION - FULL OPTIMIZED SPEED CODE
  ======================================================================

  MAIN IDEA
  ----------------------------------------------------------------------
  Gate 1 (upper buffer gate):
    - lets eggs move into buffer area
    - then closes to prevent continuous flow

  Gate 2 (lower precision gate):
    - releases only ONE egg from buffer area
    - helps avoid double egg release

  MACHINE FLOW
  ----------------------------------------------------------------------
  1. Feed cycle starts
  2. Gate1 opens briefly, then closes
  3. Gate2 opens briefly, then closes
  4. One egg slides to weighing platform
  5. System waits for egg to settle
  6. Weight is averaged and classified
  7. Pusher moves egg to pan
  8. Pan rotates to tray
  9. Tilt drops egg
 10. Pan and tilt return home
 11. Wait until scale clears
 12. Next cycle

  OPTIMIZED FOR
  ----------------------------------------------------------------------
  - faster servo motion
  - faster long-distance travel for XS and Jumbo
  - target around 12 eggs/min depending on real hardware
  - reduced delay bottlenecks
  - keeps stable weighing logic

  EGG CLASSIFICATION
  ----------------------------------------------------------------------
  XS      = 40.0 to 49.9 g
  Small   = 50.0 to 54.9 g
  Medium  = 55.0 to 59.9 g
  Large   = 60.0 to 64.9 g
  XL      = 65.0 to 69.9 g
  Jumbo   = 70.0 g and above

  SERIAL COMMANDS
  ----------------------------------------------------------------------
  s = print status
  r = reset counters
  t = tare scale
  f = feed one cycle now
  a = toggle automatic feeding
  p = pause feeding
  1 = slow speed
  2 = normal speed
  3 = fast speed
*/


// ======================================================================
// WIFI SETTINGS
// ======================================================================
const char* WIFI_SSID     = "crewbanger";
const char* WIFI_PASSWORD = "BN-2310026-2";


// ======================================================================
// WEB SERVER
// ======================================================================
WebServer server(80);


// ======================================================================
// LCD SETTINGS
// ======================================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
const uint8_t lcdCols = 16;


// ======================================================================
// HX711 SETTINGS
// ======================================================================
#define HX_DOUT 4
#define HX_SCK  5
HX711 scale;

// tune after calibration
float calibration_factor = 630.0;


// ======================================================================
// SERVO OBJECTS
// ======================================================================
Servo gate1Servo;
Servo gate2Servo;
Servo pusherServo;
Servo panServo;
Servo tiltServo;


// ======================================================================
// SERVO PINS
// ======================================================================
#define GATE1_PIN   14
#define GATE2_PIN   26
#define PUSHER_PIN  27
#define PAN_PIN     12
#define TILT_PIN    13


// ======================================================================
// GATE ANGLES
// ======================================================================
const int gate1ClosedAngle = 5;
const int gate1OpenAngle   = 45;

const int gate2ClosedAngle = 5;
const int gate2OpenAngle   = 40;


// ======================================================================
// PUSHER SETTINGS
// ======================================================================
const int pusherRestAngle = 100;
const int pusherPushAngle = 40;


// ======================================================================
// PAN SETTINGS
// ======================================================================
const int panHomeAngle = 90;
int panCurrentAngle = 90;


// ======================================================================
// TILT SETTINGS
// ======================================================================
const int tiltFlatAngle = 15;
const int tiltDefaultDropAngle = 120;
int currentTiltAngle = tiltFlatAngle;


// ======================================================================
// MACHINE TIMING
// ======================================================================
const uint8_t targetEggsPerMinute       = 12;
const unsigned long targetEggIntervalMs = 60000UL / targetEggsPerMinute;

// tuned for 12 eggs/min target depending on chute + scale settle
unsigned long feedIntervalMs          = 4800;
unsigned long gate1OpenTimeMs         = 145;
unsigned long gate1PauseAfterCloseMs  = 45;
unsigned long gate2OpenTimeMs         = 110;
unsigned long gate2PauseAfterCloseMs  = 65;

unsigned long settleDelayBeforeReadMs = 220;
unsigned long measurementWindowMs     = 650;
unsigned long measurementSampleDelayMs = 0;
unsigned long eggArrivalTimeoutMs     = 1400;
unsigned long scalePollDelayMs        = 12;

unsigned long postSortedDelayMs       = 90;
unsigned long waitRemoveTimeoutMs     = 3000;
unsigned long idleLoopDelayMs         = 10;
unsigned long webPollFriendlyDelayMs  = 5;

uint8_t liveReadSamples               = 1;
uint8_t measurementReadSamples        = 1;


// ======================================================================
// WEIGHT THRESHOLDS
// ======================================================================
float eggDetectThreshold  = 10.0;
float eggRemoveThreshold  = 5.0;
float stableSpanTolerance = 2.8;


// ======================================================================
// LCD DISPLAY SETTINGS
// ======================================================================
const char* lcdIdleLine1 = "MEGAJAVCHACK";
const char* lcdIdleLine2 = "EGG SORTER";

enum LCDViewMode {
  LCD_VIEW_STATUS,
  LCD_VIEW_IDLE,
  LCD_VIEW_WEIGHT
};

LCDViewMode lcdViewMode = LCD_VIEW_STATUS;
String lastLCDLine1 = "";
String lastLCDLine2 = "";


// ======================================================================
// WEB VARIABLES
// ======================================================================
float  webWeight        = 0.0;
String webEggType       = "Waiting";
String webStatus        = "Booting";
String webMessage       = "Starting system";
String webLastEggType   = "None";
float  webLastEggWeight = 0.0;
String webStateLabel    = "BOOT";
String webGate1Status   = "Closed";
String webGate2Status   = "Closed";
String webFeedMode      = "AUTO";
String webCycleInfo     = "Idle";
String webSpeedMode     = "FAST";


// ======================================================================
// COUNTERS
// ======================================================================
unsigned long totalEggsSorted = 0;
unsigned long totalFeedCycles = 0;
unsigned long totalReleased   = 0;
unsigned long countXS         = 0;
unsigned long countSmall      = 0;
unsigned long countMedium     = 0;
unsigned long countLarge      = 0;
unsigned long countXL         = 0;
unsigned long countJumbo      = 0;
unsigned long countUnknown    = 0;
unsigned long countFeedMiss   = 0;


// ======================================================================
// TIMING VARIABLES
// ======================================================================
unsigned long systemStartMillis = 0;
unsigned long lastSortedMillis  = 0;
unsigned long lastWiFiCheck     = 0;
unsigned long lastFeedMillis    = 0;
unsigned long lastLoopPrintMs   = 0;


// ======================================================================
// FEED CONTROL
// ======================================================================
bool autoFeedEnabled = true;
bool machineBusy     = false;
bool machinePaused   = false;
bool cycleRequested  = false;
bool allowUnknownPush = false;


// ======================================================================
// MACHINE STATE
// ======================================================================
enum MachineState {
  STATE_BOOT,
  STATE_IDLE,
  STATE_FEED_GATE1,
  STATE_FEED_GATE2,
  STATE_SETTLING,
  STATE_MEASURING,
  STATE_PUSHING,
  STATE_MOVING_PAN,
  STATE_TILTING,
  STATE_RETURNING,
  STATE_WAIT_REMOVE,
  STATE_UNKNOWN,
  STATE_PAUSED,
  STATE_ERROR
};

MachineState machineState = STATE_BOOT;

struct WeightStats {
  float average;
  float median;
  float trimmedAverage;
  float minimum;
  float maximum;
  float span;
  uint8_t count;
};


// ======================================================================
// FUNCTION DECLARATIONS
// ======================================================================
void connectToWiFi();
void reconnectWiFiIfNeeded();

void setupWebRoutes();
String buildDashboardHTML();
String buildJSONData();

void resetCounters();

void initializeLCD();
void initializeHX711();
void initializeServos();

float readWeight(uint8_t samples = 1);
void sortWeightSamples(float *values, uint8_t count);
WeightStats collectWeightStats(unsigned long durationMs, unsigned long sampleDelayMs, uint8_t samplesPerRead);
float finalizeWeight(const WeightStats &stats);

String classifyEgg(float weight);
int getPanAngle(String eggType);
int getTiltAngle(String eggType);
String getStateLabel(MachineState state);

void setMachineState(MachineState state, String statusText, String messageText);

void serviceBackground();
void updateLCDIdle(float liveWeight);
void showSystemNameOnLCD();
void writeLCDScreen(String line1, String line2, LCDViewMode nextMode);
void showWeightOnLCD(float weight, String eggType);
void showStatusOnLCD(String line1, String line2);
void printFixedLine(uint8_t col, uint8_t row, String text);

void moveServoSmooth(Servo &servo, int fromAngle, int toAngle, int stepValue, int stepDelay);
void movePanTo(int targetAngle);
void returnPanHome();
void pushEgg();
void tiltDrop(String eggType);

void closeGate1();
void openGate1();
void closeGate2();
void openGate2();

bool performDualGateFeedCycle();
bool waitForEggArrivalOnScale(unsigned long timeoutMs);
bool waitForEggRemoval();
void processEggCycle();
void tareScaleNow();

void updateCounters(String eggType);
String formatUptime();
String wifiQualityText(int rssi);
String statusClassFromText(String status);
String escapeJson(String text);
void printSerialBanner();
void printSystemStatus();
void handleSerialCommands();
float constrainPercent(float value, float maxValue);
void applySpeedProfileSlow();
void applySpeedProfileNormal();
void applySpeedProfileFast();


// ======================================================================
// SETUP
// ======================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  systemStartMillis = millis();

  initializeLCD();
  showStatusOnLCD("Booting...", "Please wait");

  connectToWiFi();
  setupWebRoutes();
  initializeHX711();
  initializeServos();
  applySpeedProfileFast();
  printSerialBanner();

  setMachineState(STATE_IDLE, "Waiting", "System ready");
  webFeedMode = autoFeedEnabled ? "AUTO" : "MANUAL";
  showSystemNameOnLCD();
}


// ======================================================================
// LOOP
// ======================================================================
void loop() {
  serviceBackground();
  handleSerialCommands();

  if (machinePaused) {
    machineBusy = false;
    setMachineState(STATE_PAUSED, "Paused", "Feeding paused");
    float pausedWeight = readWeight(liveReadSamples);
    webWeight = pausedWeight;
    webEggType = (pausedWeight > eggRemoveThreshold) ? classifyEgg(pausedWeight) : "Waiting";
    updateLCDIdle(pausedWeight);
    delay(idleLoopDelayMs);
    return;
  }

  float currentWeight = readWeight(liveReadSamples);
  webWeight = currentWeight;

  if (currentWeight <= eggRemoveThreshold) {
    webEggType = "Waiting";
  } else if (!machineBusy) {
    webEggType = classifyEgg(currentWeight);
  }

  if (!machineBusy) {
    updateLCDIdle(currentWeight);

    if (currentWeight > eggDetectThreshold) {
      processEggCycle();
    } else {
      if (cycleRequested) {
        cycleRequested = false;
        performDualGateFeedCycle();
      } else if (autoFeedEnabled && millis() - lastFeedMillis >= feedIntervalMs) {
        performDualGateFeedCycle();
      }
    }
  }

  delay(idleLoopDelayMs);
}


// ======================================================================
// BACKGROUND SERVICE
// ======================================================================
void serviceBackground() {
  server.handleClient();
  reconnectWiFiIfNeeded();
}


// ======================================================================
// WIFI
// ======================================================================
void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println();
  Serial.println("====================================");
  Serial.println("Connecting to WiFi...");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  Serial.println("====================================");

  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    lcd.clear();
    printFixedLine(0, 0, "Connecting WiFi");
    printFixedLine(0, 1, "Please wait...");

    if (millis() - startAttempt > 20000) {
      Serial.println();
      Serial.println("WiFi timeout. Retrying...");
      WiFi.disconnect(true);
      delay(1000);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      startAttempt = millis();
    }
  }

  Serial.println();
  Serial.println("WiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  lcd.clear();
  printFixedLine(0, 0, "WiFi Connected");
  printFixedLine(0, 1, WiFi.localIP().toString());
  delay(1500);
}

void reconnectWiFiIfNeeded() {
  if (millis() - lastWiFiCheck < 5000) return;
  lastWiFiCheck = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}


// ======================================================================
// WEB ROUTES
// ======================================================================
void setupWebRoutes() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", buildDashboardHTML());
  });

  server.on("/data", HTTP_GET, []() {
    server.send(200, "application/json", buildJSONData());
  });

  server.on("/reset", HTTP_GET, []() {
    resetCounters();
    server.send(200, "application/json", "{\"ok\":true,\"message\":\"Counters reset\"}");
  });

  server.on("/tare", HTTP_GET, []() {
    tareScaleNow();
    server.send(200, "application/json", "{\"ok\":true,\"message\":\"Scale tared\"}");
  });

  server.on("/feed", HTTP_GET, []() {
    cycleRequested = true;
    server.send(200, "application/json", "{\"ok\":true,\"message\":\"Feed cycle requested\"}");
  });

  server.on("/toggleFeed", HTTP_GET, []() {
    autoFeedEnabled = !autoFeedEnabled;
    webFeedMode = autoFeedEnabled ? "AUTO" : "MANUAL";
    server.send(200, "application/json", "{\"ok\":true,\"mode\":\"" + webFeedMode + "\"}");
  });

  server.on("/pause", HTTP_GET, []() {
    machinePaused = !machinePaused;
    server.send(200, "application/json", "{\"ok\":true,\"paused\":" + String(machinePaused ? "true" : "false") + "}");
  });

  server.on("/speed/slow", HTTP_GET, []() {
    applySpeedProfileSlow();
    server.send(200, "application/json", "{\"ok\":true,\"speed\":\"slow\"}");
  });

  server.on("/speed/normal", HTTP_GET, []() {
    applySpeedProfileNormal();
    server.send(200, "application/json", "{\"ok\":true,\"speed\":\"normal\"}");
  });

  server.on("/speed/fast", HTTP_GET, []() {
    applySpeedProfileFast();
    server.send(200, "application/json", "{\"ok\":true,\"speed\":\"fast\"}");
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "404 - Not Found");
  });

  server.begin();
  Serial.println("Web server started");
}


// ======================================================================
// BUILD JSON
// ======================================================================
String buildJSONData() {
  String json = "{";

  json += "\"weight\":" + String(webWeight, 1) + ",";
  json += "\"eggType\":\"" + escapeJson(webEggType) + "\",";
  json += "\"status\":\"" + escapeJson(webStatus) + "\",";
  json += "\"message\":\"" + escapeJson(webMessage) + "\",";
  json += "\"lastEggType\":\"" + escapeJson(webLastEggType) + "\",";
  json += "\"lastEggWeight\":" + String(webLastEggWeight, 1) + ",";
  json += "\"state\":\"" + escapeJson(webStateLabel) + "\",";
  json += "\"gate1\":\"" + escapeJson(webGate1Status) + "\",";
  json += "\"gate2\":\"" + escapeJson(webGate2Status) + "\",";
  json += "\"feedMode\":\"" + escapeJson(webFeedMode) + "\",";
  json += "\"cycleInfo\":\"" + escapeJson(webCycleInfo) + "\",";
  json += "\"speedMode\":\"" + escapeJson(webSpeedMode) + "\",";
  json += "\"total\":" + String(totalEggsSorted) + ",";
  json += "\"feedCycles\":" + String(totalFeedCycles) + ",";
  json += "\"released\":" + String(totalReleased) + ",";
  json += "\"feedMiss\":" + String(countFeedMiss) + ",";
  json += "\"xs\":" + String(countXS) + ",";
  json += "\"small\":" + String(countSmall) + ",";
  json += "\"medium\":" + String(countMedium) + ",";
  json += "\"large\":" + String(countLarge) + ",";
  json += "\"xl\":" + String(countXL) + ",";
  json += "\"jumbo\":" + String(countJumbo) + ",";
  json += "\"unknown\":" + String(countUnknown) + ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"wifiText\":\"" + wifiQualityText(WiFi.RSSI()) + "\",";
  json += "\"uptime\":\"" + formatUptime() + "\",";
  json += "\"panAngle\":" + String(panCurrentAngle) + ",";
  json += "\"tiltAngle\":" + String(currentTiltAngle) + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"autoFeed\":" + String(autoFeedEnabled ? "true" : "false") + ",";
  json += "\"paused\":" + String(machinePaused ? "true" : "false") + ",";
  json += "\"busy\":" + String(machineBusy ? "true" : "false") + ",";
  json += "\"intervalMs\":" + String(feedIntervalMs) + ",";
  json += "\"targetEggsPerMinute\":" + String(targetEggsPerMinute) + ",";
  json += "\"statusClass\":\"" + statusClassFromText(webStatus) + "\"";

  json += "}";
  return json;
}


// ======================================================================
// DASHBOARD HTML
// ======================================================================
String buildDashboardHTML() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1.0">
  <title>Dual Gate Egg Sorter Dashboard</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    :root{
      --bg1:#f8b236;
      --bg2:#f97316;
      --bg3:#f43f5e;
      --bg4:#facc15;
      --glass:rgba(255,255,255,.14);
      --card:#ffffff;
      --text:#111827;
      --muted:#6b7280;
      --shadow:0 18px 38px rgba(0,0,0,.18);
      --radius:22px;
    }
    body{
      min-height:100vh;
      overflow-x:hidden;
      font-family:"Trebuchet MS",Verdana,sans-serif;
      color:#fff;
      background:
        radial-gradient(circle at top left, rgba(255,255,255,.18), transparent 28%),
        linear-gradient(-45deg,var(--bg1),var(--bg2),var(--bg3),var(--bg4));
      background-size:400% 400%;
      animation:bgmove 14s ease infinite;
    }
    @keyframes bgmove{
      0%{background-position:0% 50%}
      50%{background-position:100% 50%}
      100%{background-position:0% 50%}
    }
    .floating-eggs{
      position:fixed;inset:0;overflow:hidden;pointer-events:none;z-index:0;
    }
    .float-egg{
      --egg-scale:1;
      --drift:28px;
      --rot:8deg;
      position:absolute;bottom:-180px;width:78px;height:104px;border-radius:56% 56% 48% 48%;
      background:linear-gradient(180deg,rgba(255,255,255,.88),rgba(255,245,214,.42));
      box-shadow:inset -8px -12px 0 rgba(255,255,255,.16),0 18px 30px rgba(36,22,7,.20);
      opacity:.24;animation:floatEgg 22s linear infinite;
    }
    .float-egg:after{
      content:"";position:absolute;width:24px;height:34px;border-radius:50%;
      background:rgba(255,255,255,.44);left:14px;top:14px;transform:rotate(-18deg);
    }
    .float-egg:nth-child(1){left:4%;animation-duration:19s;animation-delay:-2s;--egg-scale:.78;--drift:40px;--rot:10deg}
    .float-egg:nth-child(2){left:16%;animation-duration:24s;animation-delay:-7s;--egg-scale:1.16;--drift:-32px;--rot:-8deg}
    .float-egg:nth-child(3){left:28%;animation-duration:21s;animation-delay:-12s;--egg-scale:.92;--drift:36px;--rot:7deg}
    .float-egg:nth-child(4){left:42%;animation-duration:27s;animation-delay:-5s;--egg-scale:1.28;--drift:-24px;--rot:-10deg}
    .float-egg:nth-child(5){left:58%;animation-duration:23s;animation-delay:-10s;--egg-scale:.84;--drift:30px;--rot:8deg}
    .float-egg:nth-child(6){left:70%;animation-duration:26s;animation-delay:-4s;--egg-scale:1.08;--drift:-34px;--rot:-7deg}
    .float-egg:nth-child(7){left:82%;animation-duration:20s;animation-delay:-14s;--egg-scale:.74;--drift:28px;--rot:9deg}
    .float-egg:nth-child(8){left:92%;animation-duration:28s;animation-delay:-8s;--egg-scale:1.22;--drift:-20px;--rot:-6deg}
    @keyframes floatEgg{
      0%{transform:translate3d(0,0,0) rotate(0deg) scale(var(--egg-scale))}
      50%{transform:translate3d(var(--drift),-56vh,0) rotate(var(--rot)) scale(var(--egg-scale))}
      100%{transform:translate3d(calc(var(--drift) * -.7),-120vh,0) rotate(calc(var(--rot) * -1)) scale(var(--egg-scale))}
    }
    .container{position:relative;z-index:1;max-width:1280px;margin:0 auto;padding:18px}
    .hero{
      background:var(--glass);
      backdrop-filter:blur(14px);
      border:1px solid rgba(255,255,255,.18);
      border-radius:28px;
      box-shadow:var(--shadow);
      padding:24px;
      margin-bottom:18px;
    }
    .hero-top{
      display:flex;justify-content:space-between;gap:14px;align-items:center;flex-wrap:wrap;
    }
    .hero h1{font-size:clamp(24px,4vw,40px);margin-bottom:8px}
    .live-tag{
      display:inline-flex;gap:10px;align-items:center;
      background:rgba(0,0,0,.22);padding:12px 18px;border-radius:999px;font-weight:bold;
    }
    .dot{
      width:12px;height:12px;border-radius:50%;
      background:#22c55e;animation:pulse 1.2s infinite;
      box-shadow:0 0 0 rgba(34,197,94,.6);
    }
    @keyframes pulse{
      0%{box-shadow:0 0 0 0 rgba(34,197,94,.7)}
      70%{box-shadow:0 0 0 14px rgba(34,197,94,0)}
      100%{box-shadow:0 0 0 0 rgba(34,197,94,0)}
    }
    .hero-grid{
      margin-top:20px;
      display:grid;grid-template-columns:1.2fr .8fr;gap:18px;
    }
    .glass-box{
      background:rgba(255,255,255,.16);
      border-radius:24px;padding:20px;border:1px solid rgba(255,255,255,.14);
      min-height:220px;
    }
    .label{font-size:13px;text-transform:uppercase;letter-spacing:1.2px;opacity:.86;margin-bottom:12px}
    .weight-big{font-size:clamp(44px,6vw,74px);font-weight:900;line-height:1;margin-bottom:10px}
    .chip-row{display:flex;flex-wrap:wrap;gap:10px}
    .chip{
      display:inline-flex;align-items:center;gap:6px;
      background:rgba(255,255,255,.18);
      border-radius:999px;padding:10px 14px;font-weight:bold;border:1px solid rgba(255,255,255,.14)
    }
    .bar{width:100%;height:14px;border-radius:999px;overflow:hidden;background:rgba(255,255,255,.25);margin-top:12px}
    .bar>div{height:100%;width:0%;background:linear-gradient(90deg,#10b981,#3b82f6,#8b5cf6);transition:width .35s ease}
    .egg-wrap{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:16px;height:100%}
    .hero-egg-stage{animation:eggBounce 1.6s ease-in-out infinite}
    .hero-egg{
      --egg-scale:1;
      --egg-shell:#fff7e0;
      --egg-shadow:rgba(101,67,33,.20);
      width:calc(88px * var(--egg-scale));
      height:calc(114px * var(--egg-scale));
      background:linear-gradient(165deg,#fffdf7 0%,var(--egg-shell) 62%,#f2dfb6 100%);
      border-radius:55% 55% 50% 50%;
      box-shadow:inset -8px -14px 0 rgba(0,0,0,.04),0 18px 30px var(--egg-shadow);
      transition:width .35s ease,height .35s ease,background .35s ease,box-shadow .35s ease;
      position:relative;
    }
    .hero-egg:after{
      content:"";position:absolute;width:28px;height:38px;background:rgba(255,255,255,.55);
      border-radius:50%;left:14px;top:16px;transform:rotate(-18deg)
    }
    @keyframes eggBounce{
      0%{transform:translateY(0)}
      50%{transform:translateY(-10px)}
      100%{transform:translateY(0)}
    }
    .egg-type{font-size:32px;font-weight:900;text-align:center}
    .grid{display:grid;grid-template-columns:repeat(12,1fr);gap:18px}
    .card{
      background:var(--card);color:var(--text);border-radius:var(--radius);
      box-shadow:var(--shadow);padding:20px;overflow:hidden;
    }
    .card h3{
      font-size:15px;color:var(--muted);margin-bottom:10px;font-weight:700;
      text-transform:uppercase;letter-spacing:.9px;
    }
    .card-value{font-size:34px;font-weight:900;line-height:1.1;margin-bottom:6px}
    .card-sub{font-size:14px;color:var(--muted);line-height:1.5}
    .col-3{grid-column:span 3}
    .col-4{grid-column:span 4}
    .col-6{grid-column:span 6}
    .col-12{grid-column:span 12}
    .counter-grid{display:grid;grid-template-columns:repeat(7,1fr);gap:14px;margin-top:6px}
    .counter-box{padding:18px 10px;border-radius:18px;color:white;text-align:center;box-shadow:0 10px 25px rgba(0,0,0,.12)}
    .counter-box .name{font-size:14px;font-weight:bold;margin-bottom:8px}
    .counter-box .num{font-size:34px;font-weight:900;line-height:1}
    .xs-box{background:linear-gradient(135deg,#10b981,#34d399)}
    .sm-box{background:linear-gradient(135deg,#3b82f6,#60a5fa)}
    .md-box{background:linear-gradient(135deg,#8b5cf6,#a78bfa)}
    .lg-box{background:linear-gradient(135deg,#f59e0b,#fbbf24)}
    .xl-box{background:linear-gradient(135deg,#ef4444,#fb7185)}
    .jb-box{background:linear-gradient(135deg,#ec4899,#f472b6)}
    .uk-box{background:linear-gradient(135deg,#6b7280,#9ca3af)}
    .system-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}
    .system-item{background:#f9fafb;border-radius:18px;padding:18px;border:1px solid #eef2f7}
    .system-item .title{color:#6b7280;font-size:13px;margin-bottom:8px;font-weight:700;text-transform:uppercase;letter-spacing:.8px}
    .system-item .value{font-size:24px;font-weight:900;color:#111827;line-height:1.2;word-break:break-word}
    .wifi-bar{width:100%;height:10px;background:#e5e7eb;border-radius:999px;margin-top:10px;overflow:hidden}
    .wifi-bar>div{height:100%;width:0%;background:linear-gradient(90deg,#22c55e,#16a34a);transition:width .35s ease}
    .actions{display:flex;justify-content:flex-end;gap:12px;flex-wrap:wrap;margin-top:16px}
    .btn{
      border:none;outline:none;border-radius:14px;padding:14px 18px;cursor:pointer;
      font-weight:900;font-size:14px;transition:transform .18s ease,opacity .18s ease;color:#fff
    }
    .btn:hover{transform:translateY(-2px);opacity:.96}
    .btn-blue{background:linear-gradient(135deg,#2563eb,#1d4ed8)}
    .btn-red{background:linear-gradient(135deg,#ef4444,#dc2626)}
    .btn-purple{background:linear-gradient(135deg,#7c3aed,#6d28d9)}
    .btn-green{background:linear-gradient(135deg,#16a34a,#15803d)}
    .btn-orange{background:linear-gradient(135deg,#f59e0b,#d97706)}
    .pill{display:inline-flex;align-items:center;justify-content:center;padding:10px 16px;border-radius:999px;font-size:14px;font-weight:900;color:white;letter-spacing:.4px;min-width:110px}
    .status-waiting{background:#2563eb}
    .status-measuring{background:#f59e0b}
    .status-sorting{background:#7c3aed}
    .status-sorted{background:#16a34a}
    .status-unknown{background:#dc2626}
    .status-returning{background:#0ea5e9}
    .status-default{background:#6b7280}
    .footer{text-align:center;padding:14px 6px 26px;font-size:13px;opacity:.96}
    @media (max-width:1100px){
      .hero-grid{grid-template-columns:1fr}
      .system-grid{grid-template-columns:repeat(2,1fr)}
      .counter-grid{grid-template-columns:repeat(4,1fr)}
      .col-3,.col-4,.col-6{grid-column:span 12}
    }
    @media (max-width:720px){
      .container{padding:12px}
      .counter-grid{grid-template-columns:repeat(2,1fr)}
      .system-grid{grid-template-columns:1fr}
    }
  </style>
</head>
<body>
  <div class="floating-eggs" aria-hidden="true">
    <span class="float-egg"></span>
    <span class="float-egg"></span>
    <span class="float-egg"></span>
    <span class="float-egg"></span>
    <span class="float-egg"></span>
    <span class="float-egg"></span>
    <span class="float-egg"></span>
    <span class="float-egg"></span>
  </div>

  <div class="container">
    <div class="hero">
      <div class="hero-top">
        <div>
          <h1>MEGAJAVCHACK DUAL GATE EGG SORTER</h1>
          <p>Target throughput: 12 eggs per minute</p>
        </div>
        <div class="live-tag">
          <span class="dot"></span>
          LIVE DATA
        </div>
      </div>

      <div class="hero-grid">
        <div class="glass-box">
          <div class="label">Current Weight</div>
          <div class="weight-big"><span id="weight">0.0</span> g</div>
          <div class="chip-row">
            <div class="chip">State: <span id="state">BOOT</span></div>
            <div id="statusPill" class="pill status-default">Booting</div>
            <div class="chip">Mode: <span id="feedMode">AUTO</span></div>
            <div class="chip">Cycle: <span id="cycleInfo">Idle</span></div>
            <div class="chip">Speed: <span id="speedMode">FAST</span></div>
            <div class="chip">Target: <span id="targetRate">12 eggs/min</span></div>
          </div>
          <div class="bar"><div id="weightBar"></div></div>
          <div style="margin-top:12px;opacity:.94" id="message">Starting system...</div>
        </div>

        <div class="glass-box">
          <div class="egg-wrap">
            <div class="hero-egg-stage">
              <div id="heroEgg" class="hero-egg"></div>
            </div>
            <div class="egg-type" id="eggType">Waiting</div>
            <div class="chip-row" style="justify-content:center">
              <div class="chip">Last: <span id="lastEggType">None</span></div>
              <div class="chip"><span id="lastEggWeight">0.0</span> g</div>
              <div class="chip">G1: <span id="gate1">Closed</span></div>
              <div class="chip">G2: <span id="gate2">Closed</span></div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="grid">
      <div class="card col-3">
        <h3>Total Eggs Sorted</h3>
        <div class="card-value" id="total">0</div>
        <div class="card-sub">Total valid eggs classified and dropped.</div>
      </div>

      <div class="card col-3">
        <h3>Total Feed Cycles</h3>
        <div class="card-value" id="feedCycles">0</div>
        <div class="card-sub">Total dual-gate feed cycles executed.</div>
      </div>

      <div class="card col-3">
        <h3>Total Released</h3>
        <div class="card-value" id="released">0</div>
        <div class="card-sub">Eggs detected after gate release.</div>
      </div>

      <div class="card col-3">
        <h3>Feed Miss</h3>
        <div class="card-value" id="feedMiss">0</div>
        <div class="card-sub">Feed cycles where no egg reached the scale.</div>
      </div>

      <div class="card col-12">
        <h3>Egg Counters</h3>
        <div class="counter-grid">
          <div class="counter-box xs-box"><div class="name">XS</div><div class="num" id="xs">0</div></div>
          <div class="counter-box sm-box"><div class="name">Small</div><div class="num" id="small">0</div></div>
          <div class="counter-box md-box"><div class="name">Medium</div><div class="num" id="medium">0</div></div>
          <div class="counter-box lg-box"><div class="name">Large</div><div class="num" id="large">0</div></div>
          <div class="counter-box xl-box"><div class="name">XL</div><div class="num" id="xl">0</div></div>
          <div class="counter-box jb-box"><div class="name">Jumbo</div><div class="num" id="jumbo">0</div></div>
          <div class="counter-box uk-box"><div class="name">Unknown</div><div class="num" id="unknown">0</div></div>
        </div>
      </div>

      <div class="card col-6">
        <h3>System Information</h3>
        <div class="system-grid">
          <div class="system-item">
            <div class="title">ESP32 IP Address</div>
            <div class="value" id="ip">0.0.0.0</div>
          </div>
          <div class="system-item">
            <div class="title">Pan Position</div>
            <div class="value"><span id="panAngle">90</span>°</div>
          </div>
          <div class="system-item">
            <div class="title">Tilt Position</div>
            <div class="value"><span id="tiltAngle">15</span>°</div>
          </div>
          <div class="system-item">
            <div class="title">Free Heap</div>
            <div class="value"><span id="freeHeap">0</span></div>
          </div>
        </div>
      </div>

      <div class="card col-6">
        <h3>WiFi & Controls</h3>
        <div class="card-value"><span id="rssi">0</span> dBm</div>
        <div class="card-sub" id="wifiText">Checking signal...</div>
        <div class="wifi-bar"><div id="wifiBar"></div></div>

        <div class="actions">
          <button class="btn btn-green" onclick="feedNow()">Feed Now</button>
          <button class="btn btn-purple" onclick="toggleFeed()">Toggle Auto</button>
          <button class="btn btn-orange" onclick="togglePause()">Pause/Resume</button>
          <button class="btn btn-blue" onclick="tareScale()">Tare Scale</button>
          <button class="btn btn-red" onclick="resetCounters()">Reset Counters</button>
        </div>

        <div class="actions">
          <button class="btn btn-blue" onclick="setSlow()">Slow</button>
          <button class="btn btn-green" onclick="setNormal()">Normal</button>
          <button class="btn btn-purple" onclick="setFast()">Fast</button>
        </div>
      </div>

      <div class="card col-12">
        <h3>Live Details</h3>
        <div class="card-sub">
          Uptime: <b id="uptime">0h 0m 0s</b> &nbsp; | &nbsp;
          Target: <b id="targetRateDetail">12 eggs/min</b> &nbsp; | &nbsp;
          Interval: <b id="intervalMs">0</b> ms &nbsp; | &nbsp;
          Status: <b id="statusText">Waiting</b>
        </div>
      </div>
    </div>

    <div class="footer">
      Dashboard updates automatically without page reload.
    </div>
  </div>

  <script>
    let fetchBusy = false;

    function statusClass(status){
      const s = String(status || '').toLowerCase();
      if(s.includes('waiting')) return 'status-waiting';
      if(s.includes('measuring')) return 'status-measuring';
      if(s.includes('sorting')) return 'status-sorting';
      if(s.includes('sorted')) return 'status-sorted';
      if(s.includes('unknown')) return 'status-unknown';
      if(s.includes('returning')) return 'status-returning';
      return 'status-default';
    }

    function setText(id, value){
      const el = document.getElementById(id);
      if(el) el.textContent = value;
    }

    function setWidth(id, value){
      const el = document.getElementById(id);
      if(el) el.style.width = value;
    }

    function getEggVisualConfig(type, weight){
      const name = String(type || 'Waiting').toLowerCase();
      const fallbackScale = Math.max(0.82, Math.min(1.34, 0.78 + (Math.min(Number(weight) || 0, 80) / 140)));

      if(name === 'xs') return {scale:0.82, shell:'#fff8d9', shadow:'rgba(173,120,32,.20)'};
      if(name === 'small') return {scale:0.92, shell:'#fff4ce', shadow:'rgba(176,124,34,.22)'};
      if(name === 'medium') return {scale:1.00, shell:'#fde7be', shadow:'rgba(171,104,34,.24)'};
      if(name === 'large') return {scale:1.10, shell:'#f8ddb0', shadow:'rgba(156,86,25,.26)'};
      if(name === 'xl') return {scale:1.20, shell:'#f4d2a1', shadow:'rgba(142,74,18,.28)'};
      if(name === 'jumbo') return {scale:1.32, shell:'#eec88c', shadow:'rgba(121,63,15,.32)'};
      if(name === 'unknown') return {scale:fallbackScale, shell:'#e5e7eb', shadow:'rgba(75,85,99,.28)'};
      return {scale:0.96, shell:'#fff8e8', shadow:'rgba(123,97,52,.18)'};
    }

    function applyEggVisual(type, weight){
      const egg = document.getElementById('heroEgg');
      if(!egg) return;

      const config = getEggVisualConfig(type, weight);
      egg.style.setProperty('--egg-scale', config.scale);
      egg.style.setProperty('--egg-shell', config.shell);
      egg.style.setProperty('--egg-shadow', config.shadow);
    }

    async function fetchData(force=false){
      if(fetchBusy && !force) return;
      fetchBusy = true;
      try{
        const res = await fetch('/data?_=' + Date.now(), {cache:'no-store'});
        const data = await res.json();

        setText('weight', Number(data.weight || 0).toFixed(1));
        setText('eggType', data.eggType || 'Waiting');
        setText('statusText', data.status || 'Waiting');
        setText('message', data.message || '-');
        setText('lastEggType', data.lastEggType || 'None');
        setText('lastEggWeight', Number(data.lastEggWeight || 0).toFixed(1));
        setText('state', data.state || 'BOOT');
        setText('gate1', data.gate1 || 'Closed');
        setText('gate2', data.gate2 || 'Closed');
        setText('feedMode', data.feedMode || 'AUTO');
        setText('cycleInfo', data.cycleInfo || 'Idle');
        setText('speedMode', data.speedMode || 'FAST');
        setText('targetRate', (data.targetEggsPerMinute ?? 12) + ' eggs/min');
        setText('targetRateDetail', (data.targetEggsPerMinute ?? 12) + ' eggs/min');

        setText('total', data.total ?? 0);
        setText('feedCycles', data.feedCycles ?? 0);
        setText('released', data.released ?? 0);
        setText('feedMiss', data.feedMiss ?? 0);
        setText('xs', data.xs ?? 0);
        setText('small', data.small ?? 0);
        setText('medium', data.medium ?? 0);
        setText('large', data.large ?? 0);
        setText('xl', data.xl ?? 0);
        setText('jumbo', data.jumbo ?? 0);
        setText('unknown', data.unknown ?? 0);

        setText('ip', data.ip || '0.0.0.0');
        setText('rssi', data.rssi ?? 0);
        setText('wifiText', data.wifiText || 'Unknown');
        setText('uptime', data.uptime || '0h 0m 0s');
        setText('panAngle', data.panAngle ?? 90);
        setText('tiltAngle', data.tiltAngle ?? 15);
        setText('freeHeap', data.freeHeap ?? 0);
        setText('intervalMs', data.intervalMs ?? 0);

        const statusPill = document.getElementById('statusPill');
        if(statusPill){
          statusPill.textContent = data.status || 'Waiting';
          statusPill.className = 'pill ' + statusClass(data.status || '');
        }

        const weight = Number(data.weight || 0);
        const weightPercent = Math.max(0, Math.min(100, weight));
        setWidth('weightBar', weightPercent + '%');
        applyEggVisual(data.eggType || 'Waiting', weight);

        const rssi = Number(data.rssi || -100);
        const wifiPercent = Math.max(0, Math.min(100, ((rssi + 100) / 60) * 100));
        setWidth('wifiBar', wifiPercent + '%');
      }catch(e){
        setText('message', 'Dashboard fetch error. Check WiFi connection.');
        console.error(e);
      }finally{
        fetchBusy = false;
      }
    }

    async function resetCounters(){ await fetch('/reset'); fetchData(true); }
    async function tareScale(){ await fetch('/tare'); fetchData(true); }
    async function feedNow(){ await fetch('/feed'); fetchData(true); }
    async function toggleFeed(){ await fetch('/toggleFeed'); fetchData(true); }
    async function togglePause(){ await fetch('/pause'); fetchData(true); }
    async function setSlow(){ await fetch('/speed/slow'); fetchData(true); }
    async function setNormal(){ await fetch('/speed/normal'); fetchData(true); }
    async function setFast(){ await fetch('/speed/fast'); fetchData(true); }

    fetchData(true);
    setInterval(fetchData, 400);
  </script>
</body>
</html>
)rawliteral";

  return html;
}


// ======================================================================
// RESET COUNTERS
// ======================================================================
void resetCounters() {
  countXS = 0;
  countSmall = 0;
  countMedium = 0;
  countLarge = 0;
  countXL = 0;
  countJumbo = 0;
  countUnknown = 0;
  countFeedMiss = 0;
  totalEggsSorted = 0;
  totalFeedCycles = 0;
  totalReleased = 0;

  webLastEggType = "None";
  webLastEggWeight = 0.0;

  Serial.println("Counters reset");
}


// ======================================================================
// LCD INIT
// ======================================================================
void initializeLCD() {
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  showStatusOnLCD("Initializing", "LCD Ready");
  delay(800);
}


// ======================================================================
// HX711 INIT
// ======================================================================
void initializeHX711() {
  showStatusOnLCD("HX711 Setup", "Taring...");
  scale.begin(HX_DOUT, HX_SCK);
  delay(1000);
  scale.set_scale(calibration_factor);
  scale.tare();
  delay(1000);

  Serial.println("HX711 initialized");
  Serial.print("Calibration factor: ");
  Serial.println(calibration_factor);
}


// ======================================================================
// SERVO INIT
// ======================================================================
void initializeServos() {
  gate1Servo.setPeriodHertz(50);
  gate2Servo.setPeriodHertz(50);
  pusherServo.setPeriodHertz(50);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);

  gate1Servo.attach(GATE1_PIN, 500, 2400);
  gate2Servo.attach(GATE2_PIN, 500, 2400);
  pusherServo.attach(PUSHER_PIN, 500, 2400);
  panServo.attach(PAN_PIN, 500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);

  gate1Servo.write(gate1ClosedAngle);
  gate2Servo.write(gate2ClosedAngle);
  pusherServo.write(pusherRestAngle);
  panServo.write(panHomeAngle);
  tiltServo.write(tiltFlatAngle);

  panCurrentAngle = panHomeAngle;
  currentTiltAngle = tiltFlatAngle;
  webGate1Status = "Closed";
  webGate2Status = "Closed";

  delay(1200);

  Serial.println("Servos initialized");
  Serial.print("Gate1 closed : "); Serial.println(gate1ClosedAngle);
  Serial.print("Gate1 open   : "); Serial.println(gate1OpenAngle);
  Serial.print("Gate2 closed : "); Serial.println(gate2ClosedAngle);
  Serial.print("Gate2 open   : "); Serial.println(gate2OpenAngle);
  Serial.print("Pusher rest  : "); Serial.println(pusherRestAngle);
  Serial.print("Pan home     : "); Serial.println(panHomeAngle);
  Serial.print("Tilt flat    : "); Serial.println(tiltFlatAngle);
}


// ======================================================================
// WEIGHT READ
// ======================================================================
float readWeight(uint8_t samples) {
  if (samples < 1) samples = 1;

  float w = scale.get_units(samples);

  if (isnan(w) || isinf(w)) return 0;
  if (w < 0) w = 0;
  return w;
}


// ======================================================================
// SORT SAMPLE BUFFER
// ======================================================================
void sortWeightSamples(float *values, uint8_t count) {
  for (uint8_t i = 1; i < count; i++) {
    float current = values[i];
    int j = i - 1;

    while (j >= 0 && values[j] > current) {
      values[j + 1] = values[j];
      j--;
    }

    values[j + 1] = current;
  }
}


// ======================================================================
// ONE-PASS WEIGHT SAMPLING
// ======================================================================
WeightStats collectWeightStats(unsigned long durationMs, unsigned long sampleDelayMs, uint8_t samplesPerRead) {
  const uint8_t maxSamples = 40;
  float values[maxSamples];
  WeightStats stats = {0, 0, 0, 0, 0, 0, 0};

  unsigned long start = millis();
  float sum = 0.0;

  while (millis() - start < durationMs && stats.count < maxSamples) {
    serviceBackground();

    float w = readWeight(samplesPerRead);
    String type = classifyEgg(w);

    values[stats.count++] = w;
    sum += w;

    if (stats.count == 1 || w < stats.minimum) stats.minimum = w;
    if (stats.count == 1 || w > stats.maximum) stats.maximum = w;

    webWeight = w;
    webEggType = type;
    webMessage = "Sampling weight...";
    showWeightOnLCD(w, type);

    if (sampleDelayMs > 0) delay(sampleDelayMs);
  }

  if (stats.count == 0) {
    return stats;
  }

  stats.average = sum / stats.count;
  stats.span = stats.maximum - stats.minimum;

  sortWeightSamples(values, stats.count);

  if (stats.count % 2 == 1) {
    stats.median = values[stats.count / 2];
  } else {
    stats.median = (values[stats.count / 2] + values[(stats.count / 2) - 1]) / 2.0;
  }

  if (stats.count < 5) {
    stats.trimmedAverage = stats.average;
    return stats;
  }

  uint8_t trim = stats.count / 5;
  if (trim < 1) trim = 1;

  float trimmedSum = 0.0;
  uint8_t trimmedCount = 0;
  for (uint8_t i = trim; i < stats.count - trim; i++) {
    trimmedSum += values[i];
    trimmedCount++;
  }

  stats.trimmedAverage = (trimmedCount > 0) ? (trimmedSum / trimmedCount) : stats.average;
  return stats;
}


// ======================================================================
// FINAL WEIGHT BLEND
// ======================================================================
float finalizeWeight(const WeightStats &stats) {
  if (stats.count == 0) return 0;
  if (stats.count < 3) return stats.average;

  if (stats.span <= stableSpanTolerance) {
    return (stats.trimmedAverage * 2.0 + stats.median + stats.average) / 4.0;
  }

  return (stats.trimmedAverage + stats.median) / 2.0;
}


// ======================================================================
// CLASSIFICATION
// ======================================================================
String classifyEgg(float weight) {
  if (weight >= 40.0 && weight < 50.0) return "XS";
  if (weight >= 50.0 && weight < 55.0) return "Small";
  if (weight >= 55.0 && weight < 60.0) return "Medium";
  if (weight >= 60.0 && weight < 65.0) return "Large";
  if (weight >= 65.0 && weight < 70.0) return "XL";
  if (weight >= 70.0) return "Jumbo";
  return "Unknown";
}


// ======================================================================
// PAN ANGLES
// ======================================================================
int getPanAngle(String eggType) {
  if (eggType == "XS")     return 0;
  if (eggType == "Small")  return 30;
  if (eggType == "Medium") return 80;
  if (eggType == "Large")  return 130;
  if (eggType == "XL")     return 160;
  if (eggType == "Jumbo")  return 180;
  return panHomeAngle;
}


// ======================================================================
// TILT ANGLES
// ======================================================================
int getTiltAngle(String eggType) {
  if (eggType == "XS")     return 115;
  if (eggType == "Small")  return 118;
  if (eggType == "Medium") return 120;
  if (eggType == "Large")  return 125;
  if (eggType == "XL")     return 132;
  if (eggType == "Jumbo")  return 140;
  return tiltDefaultDropAngle;
}


// ======================================================================
// STATE LABEL
// ======================================================================
String getStateLabel(MachineState state) {
  switch (state) {
    case STATE_BOOT:        return "BOOT";
    case STATE_IDLE:        return "IDLE";
    case STATE_FEED_GATE1:  return "GATE1";
    case STATE_FEED_GATE2:  return "GATE2";
    case STATE_SETTLING:    return "SETTLE";
    case STATE_MEASURING:   return "MEASURE";
    case STATE_PUSHING:     return "PUSH";
    case STATE_MOVING_PAN:  return "PAN";
    case STATE_TILTING:     return "TILT";
    case STATE_RETURNING:   return "RETURN";
    case STATE_WAIT_REMOVE: return "CLEAR";
    case STATE_UNKNOWN:     return "UNKNOWN";
    case STATE_PAUSED:      return "PAUSED";
    case STATE_ERROR:       return "ERROR";
    default:                return "STATE";
  }
}

void setMachineState(MachineState state, String statusText, String messageText) {
  machineState = state;
  webStatus = statusText;
  webMessage = messageText;
  webStateLabel = getStateLabel(state);

  Serial.print("[STATE] ");
  Serial.print(webStateLabel);
  Serial.print(" | ");
  Serial.print(webStatus);
  Serial.print(" | ");
  Serial.println(webMessage);
}


// ======================================================================
// LCD
// ======================================================================
void updateLCDIdle(float liveWeight) {
  if (liveWeight > eggRemoveThreshold) {
    showWeightOnLCD(liveWeight, classifyEgg(liveWeight));
  } else {
    showSystemNameOnLCD();
  }
}

void showSystemNameOnLCD() {
  writeLCDScreen(lcdIdleLine1, lcdIdleLine2, LCD_VIEW_IDLE);
}

void writeLCDScreen(String line1, String line2, LCDViewMode nextMode) {
  if (lcdViewMode == nextMode && lastLCDLine1 == line1 && lastLCDLine2 == line2) {
    return;
  }

  lcd.clear();
  printFixedLine(0, 0, line1);
  printFixedLine(0, 1, line2);
  lcdViewMode = nextMode;
  lastLCDLine1 = line1;
  lastLCDLine2 = line2;
}

void showWeightOnLCD(float weight, String eggType) {
  String line1 = "W:" + String(weight, 1) + "g";
  String line2 = "Type:" + eggType;
  writeLCDScreen(line1, line2, LCD_VIEW_WEIGHT);
}

void showStatusOnLCD(String line1, String line2) {
  writeLCDScreen(line1, line2, LCD_VIEW_STATUS);
}

void printFixedLine(uint8_t col, uint8_t row, String text) {
  lcd.setCursor(col, row);

  if (text.length() < lcdCols) {
    while (text.length() < lcdCols) text += " ";
  } else if (text.length() > lcdCols) {
    text = text.substring(0, lcdCols);
  }

  lcd.print(text);
}


// ======================================================================
// SERVO HELPERS
// ======================================================================
void moveServoSmooth(Servo &servo, int fromAngle, int toAngle, int stepValue, int stepDelay) {
  if (stepValue < 1) stepValue = 1;
  if (stepDelay < 0) stepDelay = 0;

  if (fromAngle < toAngle) {
    for (int pos = fromAngle; pos <= toAngle; pos += stepValue) {
      servo.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = fromAngle; pos >= toAngle; pos -= stepValue) {
      servo.write(pos);
      delay(stepDelay);
    }
  }
  servo.write(toAngle);
}

void openGate1() {
  moveServoSmooth(gate1Servo, gate1ClosedAngle, gate1OpenAngle, 4, 2);
  webGate1Status = "Open";
}

void closeGate1() {
  moveServoSmooth(gate1Servo, gate1OpenAngle, gate1ClosedAngle, 4, 2);
  webGate1Status = "Closed";
}

void openGate2() {
  moveServoSmooth(gate2Servo, gate2ClosedAngle, gate2OpenAngle, 4, 2);
  webGate2Status = "Open";
}

void closeGate2() {
  moveServoSmooth(gate2Servo, gate2OpenAngle, gate2ClosedAngle, 4, 2);
  webGate2Status = "Closed";
}


// ======================================================================
// DUAL GATE FEED CYCLE
// ======================================================================
bool performDualGateFeedCycle() {
  if (machineBusy || machinePaused) return false;

  machineBusy = true;
  totalFeedCycles++;
  lastFeedMillis = millis();
  webCycleInfo = "Feeding";

  setMachineState(STATE_FEED_GATE1, "Waiting", "Gate1 buffer feed");
  showSystemNameOnLCD();

  openGate1();
  delay(gate1OpenTimeMs);
  closeGate1();
  delay(gate1PauseAfterCloseMs);

  setMachineState(STATE_FEED_GATE2, "Waiting", "Gate2 one-egg release");
  showSystemNameOnLCD();

  openGate2();
  delay(gate2OpenTimeMs);
  closeGate2();
  delay(gate2PauseAfterCloseMs);

  bool arrived = waitForEggArrivalOnScale(eggArrivalTimeoutMs);
  if (arrived) {
    totalReleased++;
    webCycleInfo = "Egg Arrived";
    machineBusy = false;
    return true;
  }

  countFeedMiss++;
  webCycleInfo = "No Egg Arrived";
  webWeight = 0;
  webEggType = "Waiting";
  setMachineState(STATE_IDLE, "Waiting", "Feed cycle ended with no egg on scale");
  showSystemNameOnLCD();
  machineBusy = false;
  return false;
}

bool waitForEggArrivalOnScale(unsigned long timeoutMs) {
  unsigned long start = millis();

  setMachineState(STATE_SETTLING, "Waiting", "Watching for egg arrival");

  while (millis() - start < timeoutMs) {
    serviceBackground();
    float w = readWeight(liveReadSamples);
    webWeight = w;
    webEggType = (w > eggRemoveThreshold) ? classifyEgg(w) : "Waiting";
    updateLCDIdle(w);

    if (w > eggDetectThreshold) {
      return true;
    }

    delay(scalePollDelayMs);
  }

  return false;
}


// ======================================================================
// PROCESS EGG
// ======================================================================
void processEggCycle() {
  if (machineBusy) return;
  machineBusy = true;

  setMachineState(STATE_SETTLING, "Measuring", "Settling egg before weighing");
  float liveWeight = readWeight(liveReadSamples);
  webWeight = liveWeight;
  webEggType = (liveWeight > eggRemoveThreshold) ? classifyEgg(liveWeight) : "Waiting";
  updateLCDIdle(liveWeight);
  webCycleInfo = "Settling";

  delay(settleDelayBeforeReadMs);

  WeightStats stats = collectWeightStats(measurementWindowMs, measurementSampleDelayMs, measurementReadSamples);
  float finalWeight = finalizeWeight(stats);

  String eggType = classifyEgg(finalWeight);

  webWeight  = finalWeight;
  webEggType = eggType;
  showWeightOnLCD(finalWeight, eggType);

  Serial.println("================================================");
  Serial.print("Samples      : "); Serial.println(stats.count);
  Serial.print("Average      : "); Serial.println(stats.average, 1);
  Serial.print("Median       : "); Serial.println(stats.median, 1);
  Serial.print("Trimmed Avg  : "); Serial.println(stats.trimmedAverage, 1);
  Serial.print("Span         : "); Serial.println(stats.span, 1);
  Serial.print("Final Wt     : "); Serial.println(finalWeight, 1);
  Serial.print("Type         : "); Serial.println(eggType);
  Serial.println("================================================");

  if (eggType != "Unknown") {
    updateCounters(eggType);
    webLastEggType = eggType;
    webLastEggWeight = finalWeight;
    lastSortedMillis = millis();
    webCycleInfo = "Sorting";

    setMachineState(STATE_PUSHING, "Sorting", "Pushing egg");
    pushEgg();

    setMachineState(STATE_MOVING_PAN, "Sorting", "Moving pan");
    movePanTo(getPanAngle(eggType));

    setMachineState(STATE_TILTING, "Sorting", "Tilting egg");
    tiltDrop(eggType);

    setMachineState(STATE_RETURNING, "Returning", "Returning pan and tilt");
    currentTiltAngle = tiltFlatAngle;
    tiltServo.write(tiltFlatAngle);
    delay(90);
    returnPanHome();

    setMachineState(STATE_WAIT_REMOVE, "Sorted", "Waiting for scale to clear");
    showSystemNameOnLCD();
    delay(postSortedDelayMs);

    waitForEggRemoval();
    webCycleInfo = "Cycle Done";
    setMachineState(STATE_IDLE, "Waiting", "Ready for next egg");
  } else {
    countUnknown++;
    webLastEggType = "Unknown";
    webLastEggWeight = finalWeight;
    webCycleInfo = "Unknown";

    setMachineState(STATE_UNKNOWN, "Unknown", "Weight out of range");
    delay(350);

    waitForEggRemoval();
    setMachineState(STATE_IDLE, "Waiting", "Ready for next egg");
  }

  machineBusy = false;
}


// ======================================================================
// PUSHER
// ======================================================================
void pushEgg() {
  moveServoSmooth(pusherServo, pusherRestAngle, pusherPushAngle, 8, 1);
  delay(60);
  moveServoSmooth(pusherServo, pusherPushAngle, pusherRestAngle, 8, 1);
  delay(50);
}


// ======================================================================
// PAN
// ======================================================================
void movePanTo(int targetAngle) {
  int distance = abs(targetAngle - panCurrentAngle);

  int stepValue = 4;
  int stepDelay = 3;

  if (distance > 100) {
    stepValue = 8;
    stepDelay = 1;
  } else if (distance > 60) {
    stepValue = 6;
    stepDelay = 2;
  } else if (distance > 30) {
    stepValue = 5;
    stepDelay = 2;
  }

  moveServoSmooth(panServo, panCurrentAngle, targetAngle, stepValue, stepDelay);
  panCurrentAngle = targetAngle;
}

void returnPanHome() {
  int targetAngle = panHomeAngle;
  int distance = abs(targetAngle - panCurrentAngle);

  int stepValue = 5;
  int stepDelay = 2;

  if (distance > 100) {
    stepValue = 8;
    stepDelay = 1;
  } else if (distance > 60) {
    stepValue = 6;
    stepDelay = 2;
  }

  moveServoSmooth(panServo, panCurrentAngle, targetAngle, stepValue, stepDelay);
  panCurrentAngle = targetAngle;
}


// ======================================================================
// TILT
// ======================================================================
void tiltDrop(String eggType) {
  int tiltTarget = getTiltAngle(eggType);

  int tiltStep = 5;
  int tiltDelayValue = 2;

  if (eggType == "XS" || eggType == "Jumbo") {
    tiltStep = 10;
    tiltDelayValue = 1;
  } else if (eggType == "XL") {
    tiltStep = 7;
    tiltDelayValue = 1;
  }

  moveServoSmooth(tiltServo, tiltFlatAngle, tiltTarget, tiltStep, tiltDelayValue);
  currentTiltAngle = tiltTarget;
  delay(120);
  moveServoSmooth(tiltServo, tiltTarget, tiltFlatAngle, tiltStep, tiltDelayValue);
  currentTiltAngle = tiltFlatAngle;
  delay(50);
}


// ======================================================================
// WAIT REMOVE
// ======================================================================
bool waitForEggRemoval() {
  unsigned long startWait = millis();

  while (millis() - startWait < waitRemoveTimeoutMs) {
    serviceBackground();

    float w = readWeight(liveReadSamples);
    webWeight = w;
    webEggType = (w > eggRemoveThreshold) ? classifyEgg(w) : "Waiting";
    webMessage = "Waiting for scale to clear";
    updateLCDIdle(w);

    if (w <= eggRemoveThreshold) {
      return true;
    }

    delay(scalePollDelayMs);
  }

  return false;
}


// ======================================================================
// TARE
// ======================================================================
void tareScaleNow() {
  showStatusOnLCD("Taring Scale", "Please wait");
  webMessage = "Taring scale...";
  delay(300);
  scale.tare();
  delay(700);
  webWeight = 0;
  webEggType = "Waiting";
  webMessage = "Scale tared";
  showSystemNameOnLCD();
  Serial.println("Scale tared");
}


// ======================================================================
// COUNTERS
// ======================================================================
void updateCounters(String eggType) {
  if (eggType == "XS")     countXS++;
  if (eggType == "Small")  countSmall++;
  if (eggType == "Medium") countMedium++;
  if (eggType == "Large")  countLarge++;
  if (eggType == "XL")     countXL++;
  if (eggType == "Jumbo")  countJumbo++;

  totalEggsSorted++;
}


// ======================================================================
// SPEED PROFILES
// ======================================================================
void applySpeedProfileSlow() {
  feedIntervalMs          = 6000;
  gate1OpenTimeMs         = 260;
  gate1PauseAfterCloseMs  = 150;
  gate2OpenTimeMs         = 220;
  gate2PauseAfterCloseMs  = 160;
  settleDelayBeforeReadMs = 420;
  measurementWindowMs     = 1100;
  measurementSampleDelayMs = 20;
  eggArrivalTimeoutMs     = 1800;
  scalePollDelayMs        = 30;
  postSortedDelayMs       = 140;
  waitRemoveTimeoutMs     = 4500;
  idleLoopDelayMs         = 20;
  liveReadSamples         = 1;
  measurementReadSamples  = 1;
  webSpeedMode = "SLOW";
  Serial.println("Speed profile: SLOW");
}

void applySpeedProfileNormal() {
  feedIntervalMs          = 5200;
  gate1OpenTimeMs         = 190;
  gate1PauseAfterCloseMs  = 90;
  gate2OpenTimeMs         = 150;
  gate2PauseAfterCloseMs  = 100;
  settleDelayBeforeReadMs = 320;
  measurementWindowMs     = 850;
  measurementSampleDelayMs = 8;
  eggArrivalTimeoutMs     = 1600;
  scalePollDelayMs        = 18;
  postSortedDelayMs       = 110;
  waitRemoveTimeoutMs     = 3800;
  idleLoopDelayMs         = 14;
  liveReadSamples         = 1;
  measurementReadSamples  = 1;
  webSpeedMode = "NORMAL";
  Serial.println("Speed profile: NORMAL");
}

void applySpeedProfileFast() {
  feedIntervalMs          = 4800;
  gate1OpenTimeMs         = 145;
  gate1PauseAfterCloseMs  = 45;
  gate2OpenTimeMs         = 110;
  gate2PauseAfterCloseMs  = 65;
  settleDelayBeforeReadMs = 220;
  measurementWindowMs     = 650;
  measurementSampleDelayMs = 0;
  eggArrivalTimeoutMs     = 1400;
  scalePollDelayMs        = 12;
  postSortedDelayMs       = 90;
  waitRemoveTimeoutMs     = 3000;
  idleLoopDelayMs         = 10;
  liveReadSamples         = 1;
  measurementReadSamples  = 1;
  webSpeedMode = "FAST";
  Serial.println("Speed profile: FAST");
}


// ======================================================================
// UTILITIES
// ======================================================================
String formatUptime() {
  unsigned long seconds = (millis() - systemStartMillis) / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours   = minutes / 60;

  seconds = seconds % 60;
  minutes = minutes % 60;

  String out = "";
  out += String(hours);
  out += "h ";
  out += String(minutes);
  out += "m ";
  out += String(seconds);
  out += "s";
  return out;
}

String wifiQualityText(int rssi) {
  if (rssi >= -55) return "Excellent";
  if (rssi >= -67) return "Good";
  if (rssi >= -75) return "Fair";
  if (rssi >= -85) return "Weak";
  return "Very Weak";
}

String statusClassFromText(String status) {
  String s = status;
  s.toLowerCase();

  if (s.indexOf("waiting") >= 0) return "waiting";
  if (s.indexOf("measuring") >= 0) return "measuring";
  if (s.indexOf("sorting") >= 0) return "sorting";
  if (s.indexOf("sorted") >= 0) return "sorted";
  if (s.indexOf("unknown") >= 0) return "unknown";
  if (s.indexOf("returning") >= 0) return "returning";
  return "default";
}

String escapeJson(String text) {
  text.replace("\\", "\\\\");
  text.replace("\"", "\\\"");
  text.replace("\n", "\\n");
  text.replace("\r", "");
  return text;
}

float constrainPercent(float value, float maxValue) {
  if (maxValue <= 0) return 0;
  if (value < 0) value = 0;
  if (value > maxValue) value = maxValue;
  return (value / maxValue) * 100.0;
}


// ======================================================================
// SERIAL
// ======================================================================
void handleSerialCommands() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  if (cmd == 's' || cmd == 'S') {
    printSystemStatus();
  } else if (cmd == 'r' || cmd == 'R') {
    resetCounters();
  } else if (cmd == 't' || cmd == 'T') {
    tareScaleNow();
  } else if (cmd == 'f' || cmd == 'F') {
    cycleRequested = true;
  } else if (cmd == 'a' || cmd == 'A') {
    autoFeedEnabled = !autoFeedEnabled;
    webFeedMode = autoFeedEnabled ? "AUTO" : "MANUAL";
    Serial.print("Auto feed: ");
    Serial.println(webFeedMode);
  } else if (cmd == 'p' || cmd == 'P') {
    machinePaused = !machinePaused;
    Serial.print("Paused: ");
    Serial.println(machinePaused ? "YES" : "NO");
  } else if (cmd == '1') {
    applySpeedProfileSlow();
  } else if (cmd == '2') {
    applySpeedProfileNormal();
  } else if (cmd == '3') {
    applySpeedProfileFast();
  }
}

void printSystemStatus() {
  Serial.println();
  Serial.println("--------------- SYSTEM STATUS ---------------");
  Serial.print("Weight        : "); Serial.println(readWeight(liveReadSamples), 1);
  Serial.print("Egg Type      : "); Serial.println(webEggType);
  Serial.print("Status        : "); Serial.println(webStatus);
  Serial.print("State         : "); Serial.println(webStateLabel);
  Serial.print("Gate1         : "); Serial.println(webGate1Status);
  Serial.print("Gate2         : "); Serial.println(webGate2Status);
  Serial.print("Feed Mode     : "); Serial.println(webFeedMode);
  Serial.print("Cycle Info    : "); Serial.println(webCycleInfo);
  Serial.print("Speed Mode    : "); Serial.println(webSpeedMode);
  Serial.print("Total Sorted  : "); Serial.println(totalEggsSorted);
  Serial.print("Feed Cycles   : "); Serial.println(totalFeedCycles);
  Serial.print("Released      : "); Serial.println(totalReleased);
  Serial.print("Feed Miss     : "); Serial.println(countFeedMiss);
  Serial.print("WiFi RSSI     : "); Serial.println(WiFi.RSSI());
  Serial.print("IP            : "); Serial.println(WiFi.localIP());
  Serial.println("---------------------------------------------");
}

void printSerialBanner() {
  Serial.println();
  Serial.println("############################################################");
  Serial.println("#        MEGAJAVCHACK DUAL GATE EGG SORTER                 #");
  Serial.println("#                  FULL OPTIMIZED SPEED                    #");
  Serial.println("############################################################");
  Serial.print  ("# WiFi SSID   : "); Serial.println(WIFI_SSID);
  Serial.print  ("# IP Address  : "); Serial.println(WiFi.localIP());
  Serial.print  ("# Gate1 Pin   : "); Serial.println(GATE1_PIN);
  Serial.print  ("# Gate2 Pin   : "); Serial.println(GATE2_PIN);
  Serial.print  ("# Pusher Pin  : "); Serial.println(PUSHER_PIN);
  Serial.print  ("# Pan Pin     : "); Serial.println(PAN_PIN);
  Serial.print  ("# Tilt Pin    : "); Serial.println(TILT_PIN);
  Serial.print  ("# Feed Intvl  : "); Serial.println(feedIntervalMs);
  Serial.print  ("# Threshold   : "); Serial.println(eggDetectThreshold);
  Serial.print  ("# Calibration : "); Serial.println(calibration_factor);
  Serial.println("# Commands    : s=status r=reset t=tare f=feed a=auto p=pause");
  Serial.println("# Speed Keys  : 1=slow 2=normal 3=fast");
  Serial.println("############################################################");
}
