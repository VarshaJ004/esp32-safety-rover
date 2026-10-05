/*
 * ==============================================================================
 * ESP32 Autonomous & Manual Safety Rover Controller Firmware
 * ==============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <esp_arduino_version.h>
#include <Adafruit_NeoPixel.h>

// ======================== HARDWARE PIN DEFINITIONS ========================
#define L_RPWM          19   
#define L_LPWM          18   
#define L_REN           21   
#define L_LEN           22   

#define R_RPWM          25   
#define R_LPWM          23   
#define R_REN           27   
#define R_LEN           32   

// --- Alcohol Sensor & Buzzer ---
#define MQ3_PIN         34   
#define BUZZER_PIN      4    
#define ALCOHOL_THRESH  1400 

// --- 3-Phase Auto-Park Trajectory Settings ---
#define PARK_SPEED          135   
#define PARK_PHASE1_MS      1600  // Arc left
#define PARK_PHASE2_MS      1000  // Straighten parallel to curb
#define PARK_PHASE3_MS      600   // Soft brake to a halt
#define PARK_MIN_SIDE_CM    20    
#define PARK_CURB_CM        10    

// --- Primary WS2812B NeoPixel Setup (Signals & Indicators) ---
#define LED_PIN             5     
#define NUM_LEDS            20    
#define LED_BRIGHTNESS      120  
#define REAR_CHASE_REVERSED true  

Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- Secondary WS2812B NeoPixel Setup (Pin 14 Neon Blue Accent) ---
#define LED_PIN_2           14    
#define NUM_LEDS_2          8     

Adafruit_NeoPixel leds2(NUM_LEDS_2, LED_PIN_2, NEO_GRB + NEO_KHZ800);

// --- PWM Configurations ---
#define PWM_FREQ        5000 
#define PWM_RES         8    

#define L_RPWM_CH       0
#define L_LPWM_CH       1
#define R_RPWM_CH       2
#define R_LPWM_CH       3

// ======================== DEFAULT OPERATIONAL VALUES ======================
int manualSpeed          = 180;
int turnSpeed            = 155;
unsigned long watchdogMs = 850; 
bool autoBrakeEnabled    = false;
int obstacleBrakeDist    = 18;

// ======================== COMMON WI-FI NETWORK CONFIG =====================
const char *ssid        = "Asianet-WIFI";       
const char *password    = "200C86E89280";       

WebServer server(80);
WiFiUDP udpServer;
const unsigned int UDP_PORT = 4210;

// ======================== STATE MACHINE & TELEMETRY =======================
enum CarState {
  STATE_IDLE,
  STATE_MANUAL,
  STATE_HALTED,
  STATE_PARKING
};

volatile CarState currentState = STATE_IDLE;
volatile long frontDist        = 200;
volatile long leftDist         = 200;

volatile int currentAlcoholVal = 0;
volatile bool alcoholLock      = false;

// Hardware Noise Filters
int alcoholStrikeCount             = 0;
unsigned long ignoreAlcoholUntil   = 0; 

int targetLeft   = 0;
int targetRight  = 0;
int currentLeft  = 0;
int currentRight = 0;

unsigned long lastSensorTick   = 0;
unsigned long lastRampTick     = 0;
unsigned long lastHeartbeat    = 0;
unsigned long lastAlcoholTick  = 0;
unsigned long lastSerialDebug  = 0;
unsigned long parkStartMs      = 0;

unsigned long greenClearUntil  = 0;
bool wasInRedAlert             = false;

// ======================== LED CONTROL ROUTINES ===========================
unsigned long lastLedTick = 0;
int ledChasePosition      = 0;
bool redBlinkState        = false;

void ledsOff() { leds.clear(); leds.show(); }

void ledsWhite() {
  for (int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(255, 255, 255));
  leds.show();
}

void ledsGreen() {
  for (int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0, 255, 0));
  leds.show();
}

void drawChase(int frontBase, int rearBase) {
  leds.clear();
  int headPos  = ledChasePosition;
  int tailPos  = (ledChasePosition + 4) % 5;
  int rearHead = REAR_CHASE_REVERSED ? (4 - headPos) : headPos;
  int rearTail = REAR_CHASE_REVERSED ? (4 - tailPos) : tailPos;

  leds.setPixelColor(frontBase + headPos, leds.Color(255, 90, 0));
  leds.setPixelColor(frontBase + tailPos, leds.Color(100, 35, 0));
  leds.setPixelColor(rearBase + rearHead, leds.Color(255, 90, 0));
  leds.setPixelColor(rearBase + rearTail, leds.Color(100, 35, 0));
  leds.show();
}

void ledsLeftIndicator()  { drawChase(5, 10); } 
void ledsRightIndicator() { drawChase(0, 15); } 

void ledsBlinkRed() {
  if (redBlinkState) {
    for (int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(255, 0, 0));
  } else {
    leds.clear();
  }
  leds.show();
}

void updateLEDs() {
  unsigned long now = millis();
  bool inRedAlert = (alcoholLock || currentState == STATE_HALTED);

  // --- D14 Accent Strip: Neon Blue when ACTIVE, OFF when HALTED / LOCKED ---
  static int lastLeds2State = -1;
  int currentLeds2State = inRedAlert ? 0 : 1;

  if (currentLeds2State != lastLeds2State) {
    lastLeds2State = currentLeds2State;
    if (currentLeds2State == 1) {
      for (int i = 0; i < NUM_LEDS_2; i++) {
        leds2.setPixelColor(i, leds2.Color(0, 210, 255)); // Neon Cyan/Blue
      }
    } else {
      leds2.clear();
    }
    leds2.show();
  }

  // --- Primary LED Strip State Machine ---
  if (inRedAlert) {
    wasInRedAlert = true;
    if (now - lastLedTick >= 125) {
      lastLedTick = now;
      redBlinkState = !redBlinkState;
      ledsBlinkRed();
      digitalWrite(BUZZER_PIN, redBlinkState ? HIGH : LOW);
    }
    return;
  }

  if (wasInRedAlert) {
    wasInRedAlert = false;
    greenClearUntil = now + 3000;
  }

  digitalWrite(BUZZER_PIN, LOW);
  redBlinkState = false;

  if (now < greenClearUntil) {
    ledsGreen();
    return;
  }

  if (currentState == STATE_PARKING) {
    if (now - lastLedTick >= 90) {
      lastLedTick = now;
      ledChasePosition = (ledChasePosition + 1) % 5;
      ledsLeftIndicator();
    }
    return;
  }

  bool isTurningLeft = (targetLeft < 0 && targetRight > 0) ||
                       (targetLeft < 0 && targetRight < 0 && targetRight < targetLeft) ||
                       (targetLeft > 0 && targetRight > 0 && targetRight > targetLeft);

  bool isTurningRight = (targetLeft > 0 && targetRight < 0) ||
                         (targetLeft < 0 && targetRight < 0 && targetLeft < targetRight) ||
                         (targetLeft > 0 && targetRight > 0 && targetLeft > targetRight);

  if (currentState == STATE_MANUAL && isTurningLeft) {
    if (now - lastLedTick >= 90) {
      lastLedTick = now;
      ledChasePosition = (ledChasePosition + 1) % 5;
      ledsLeftIndicator();
    }
    return;
  }

  if (currentState == STATE_MANUAL && isTurningRight) {
    if (now - lastLedTick >= 90) {
      lastLedTick = now;
      ledChasePosition = (ledChasePosition + 1) % 5;
      ledsRightIndicator();
    }
    return;
  }

  ledsWhite();
}

// ======================== HARDWARE PWM ABSTRACTIONS =======================
void writePwm(uint8_t pin, uint8_t channel, int value) {
  value = constrain(value, 0, 255);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(pin, value);
#else
  ledcWrite(channel, value);
#endif
}

void initPwmPin(uint8_t pin, uint8_t channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(pin, PWM_FREQ, PWM_RES);
#else
  ledcSetup(channel, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, channel);
#endif
}

void applyMotors(int leftSpeed, int rightSpeed) {
  // 1. Emergency Halt or Lock: Fully disable drivers
  if (alcoholLock || currentState == STATE_HALTED) {
    digitalWrite(L_REN, LOW);
    digitalWrite(L_LEN, LOW);
    digitalWrite(R_REN, LOW);
    digitalWrite(R_LEN, LOW);
    writePwm(L_RPWM, L_RPWM_CH, 0);
    writePwm(L_LPWM, L_LPWM_CH, 0);
    writePwm(R_RPWM, R_RPWM_CH, 0);
    writePwm(R_LPWM, R_LPWM_CH, 0);
    return;
  }

  // 2. Coast Motors (Freewheeling) when target speed is 0
  if (leftSpeed == 0 && rightSpeed == 0) {
    digitalWrite(L_REN, LOW);
    digitalWrite(L_LEN, LOW);
    digitalWrite(R_REN, LOW);
    digitalWrite(R_LEN, LOW);
    writePwm(L_RPWM, L_RPWM_CH, 0);
    writePwm(L_LPWM, L_LPWM_CH, 0);
    writePwm(R_RPWM, R_RPWM_CH, 0);
    writePwm(R_LPWM, R_LPWM_CH, 0);
    return;
  }

  // 3. Normal Driving: Enable drivers and apply PWM
  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);
  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);

  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) {
    writePwm(L_RPWM, L_RPWM_CH, leftSpeed);
    writePwm(L_LPWM, L_LPWM_CH, 0);
  } else {
    writePwm(L_RPWM, L_RPWM_CH, 0);
    writePwm(L_LPWM, L_LPWM_CH, abs(leftSpeed));
  }

  if (rightSpeed >= 0) {
    writePwm(R_RPWM, R_RPWM_CH, rightSpeed);
    writePwm(R_LPWM, R_LPWM_CH, 0);
  } else {
    writePwm(R_RPWM, R_RPWM_CH, 0);
    writePwm(R_LPWM, R_LPWM_CH, abs(rightSpeed));
  }
}

void disconnectTyres() {
  targetLeft   = 0;
  targetRight  = 0;
  currentLeft  = 0;
  currentRight = 0;
  applyMotors(0, 0);
}

void armMotorsHardware() {
  targetLeft   = 0;
  targetRight  = 0;
  currentLeft  = 0;
  currentRight = 0;
  applyMotors(0, 0); 
}

void updateMotorRamp() {
  const int RAMP_STEP = 25;

  if (currentLeft < targetLeft) currentLeft = min(currentLeft + RAMP_STEP, targetLeft);
  else if (currentLeft > targetLeft) currentLeft = max(currentLeft - RAMP_STEP, targetLeft);

  if (currentRight < targetRight) currentRight = min(currentRight + RAMP_STEP, targetRight);
  else if (currentRight > targetRight) currentRight = max(currentRight - RAMP_STEP, targetRight);

  applyMotors(currentLeft, currentRight);
}

// ======================== AUTO-PARK CONTROL ==============================
void finishAutoPark() {
  disconnectTyres();
  currentState = STATE_HALTED;
  Serial.println("[PARK] Auto-park complete. Car fully stopped and locked. Manual Re-Arm required.");
}

void startAutoPark() {
  if (currentState == STATE_HALTED || currentState == STATE_PARKING) return;

  bool moving = (currentLeft != 0 || currentRight != 0 || targetLeft != 0 || targetRight != 0);
  bool reversing = (currentLeft > 0 && currentRight > 0);

  if (!moving || reversing || alcoholLock) {
    finishAutoPark();
    return;
  }

  parkStartMs  = millis();
  currentState = STATE_PARKING;
  
  targetLeft   = -PARK_SPEED; 
  targetRight  = -PARK_SPEED / 4;     
  Serial.println("[PARK] Microsleep detected: Starting left auto-park maneuver...");
}

// ======================== SAFE BOOT PRE-CHECK =============================
void performAlcoholPreCheck() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(MQ3_PIN, INPUT);

  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);

  for (int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(255, 120, 0));
  leds.show();

  Serial.println("[BOOT] Sampling baseline MQ-3 air reading...");
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(MQ3_PIN);
    delay(50);
  }
  currentAlcoholVal = sum / 20;

  if (currentAlcoholVal >= ALCOHOL_THRESH) {
    alcoholLock = true;
    wasInRedAlert = true;
    Serial.println("[BOOT] Alcohol detected on boot! Ignition locked.");
  } else {
    alcoholLock = false;
    greenClearUntil = millis() + 3000;
    ledsGreen();
    Serial.println("[BOOT] Air clean. System Armed (3s Green Confirmation).");
  }
}

// ======================== HTTP API & CORS HELPERS =========================
void setCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, X-Requested-With, Cache-Control");
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
}

void handleOptions() { setCorsHeaders(); server.send(204); }

void handlePing() {
  setCorsHeaders();
  server.send(200, "application/json", "{\"pong\":true,\"uptime\":" + String(millis() / 1000) + "}");
}

void handleStatus() {
  setCorsHeaders();
  String st = "IDLE";
  if (alcoholLock) st = "ALCOHOL_LOCKED";
  else if (currentState == STATE_PARKING) st = "PARKING";
  else if (currentState == STATE_HALTED) st = "HALTED";
  else if (currentState == STATE_MANUAL) st = "MANUAL";

  unsigned long remainingWatchdog = 0;
  if (currentState == STATE_MANUAL) {
    unsigned long elapsed = millis() - lastHeartbeat;
    if (elapsed < watchdogMs) remainingWatchdog = watchdogMs - elapsed;
  }

  String json = "{";
  json += "\"front\":" + String(frontDist) + ",";
  json += "\"left\":" + String(leftDist) + ",";
  json += "\"state\":\"" + st + "\",";
  json += "\"alcohol\":" + String(currentAlcoholVal) + ",";
  json += "\"threshold\":" + String(ALCOHOL_THRESH) + ",";
  json += "\"alcoholDetected\":" + String(alcoholLock ? "true" : "false") + ",";
  json += "\"speed\":" + String(manualSpeed) + ",";
  json += "\"turnSpeed\":" + String(turnSpeed) + ",";
  json += "\"targetLeft\":" + String(targetLeft) + ",";
  json += "\"targetRight\":" + String(targetRight) + ",";
  json += "\"currentLeft\":" + String(currentLeft) + ",";
  json += "\"currentRight\":" + String(currentRight) + ",";
  String currentIp = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  String wifiModeStr = (WiFi.status() == WL_CONNECTED) ? "STA" : "AP";
  json += "\"clients\":0,";
  json += "\"uptime\":" + String(millis() / 1000) + ",";
  json += "\"watchdog\":" + String(remainingWatchdog) + ",";
  json += "\"autoBrake\":" + String(autoBrakeEnabled ? "true" : "false") + ",";
  json += "\"ip\":\"" + currentIp + "\",";
  json += "\"ssid\":\"" + String(ssid) + "\",";
  json += "\"wifiMode\":\"" + wifiModeStr + "\",";
  json += "\"mdns\":\"http://esp32-rover.local\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleDrive() {
  setCorsHeaders();
  lastHeartbeat = millis();

  if (alcoholLock) {
    disconnectTyres();
    server.send(403, "application/json", "{\"error\":\"Ignition locked.\"}");
    return;
  }

  if (currentState == STATE_PARKING) {
    server.send(403, "application/json", "{\"error\":\"Auto-parking.\"}");
    return;
  }

  if (currentState == STATE_HALTED) {
    disconnectTyres();
    server.send(403, "application/json", "{\"error\":\"Vehicle halted.\"}");
    return;
  }

  if (server.hasArg("left") && server.hasArg("right")) {
    int reqLeft  = -server.arg("left").toInt();
    int reqRight = -server.arg("right").toInt();

    reqLeft  = constrain(reqLeft, -255, 255);
    reqRight = constrain(reqRight, -255, 255);

    targetLeft   = reqLeft;
    targetRight  = reqRight;
    currentState = (targetLeft == 0 && targetRight == 0) ? STATE_IDLE : STATE_MANUAL;
    server.send(200, "application/json", "{\"status\":\"OK\"}");
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing params\"}");
  }
}

void handleCommand() {
  setCorsHeaders();

  if (!server.hasArg("val")) {
    server.send(400, "text/plain", "Missing val");
    return;
  }

  String cmd = server.arg("val");
  cmd.trim();
  cmd.toUpperCase();
  lastHeartbeat = millis();

  Serial.print("[HTTP CMD] val = ");
  Serial.println(cmd);

  if (server.hasArg("spd")) {
    int s = server.arg("spd").toInt();
    if (s >= 50 && s <= 255) manualSpeed = s;
  }

  if (cmd == "STOP" || cmd == "PARK" || cmd == "SLEEP") {
    startAutoPark();
    server.send(200, "text/plain", currentState == STATE_PARKING ? "Auto-Parking" : "Parked");
    return;
  }

  // --- RE-ARM / RE-ENABLE ROUTINE ---
  if (cmd == "ARM" || cmd == "RESET" || cmd == "RESTORE" || 
      cmd == "ENABLE" || cmd == "RE-ENABLE" || cmd == "REARM" || 
      cmd == "RE-ARM" || cmd == "START" || cmd == "RESUME") {
    if (alcoholLock) {
      disconnectTyres();
      server.send(403, "text/plain", "Cannot restore: Alcohol");
      Serial.println("[ARM REJECTED] Alcohol Lock Active");
      return;
    }
    
    ignoreAlcoholUntil = millis() + 2500;
    
    currentState = STATE_IDLE; 
    armMotorsHardware();
    lastHeartbeat = millis();
    server.send(200, "text/plain", "System Armed");
    Serial.println(">>> [ARMED] System restored to IDLE successfully! <<<"); 
    return;
  }

  // --- STOP COMMAND ROUTINE (ALSO RESTORES TO IDLE IF HALTED) ---
  if (cmd == "S") {
    targetLeft  = 0;
    targetRight = 0;
    
    // Automatically restore to IDLE if coming from HALTED or MANUAL
    if (!alcoholLock && (currentState == STATE_HALTED || currentState == STATE_MANUAL)) {
      currentState = STATE_IDLE;
      armMotorsHardware();
      Serial.println(">>> [ARMED VIA S] Car restored to IDLE <<<");
    } else {
      currentState = STATE_IDLE;
    }

    server.send(200, "text/plain", "OK");
    return;
  }

  if (currentState == STATE_HALTED || currentState == STATE_PARKING || alcoholLock) {
    disconnectTyres();
    server.send(403, "text/plain", "Locked");
    return;
  }

  if (cmd == "F") {
    targetLeft  = -manualSpeed;
    targetRight = -manualSpeed;
    currentState = STATE_MANUAL;
  } else if (cmd == "B") {
    targetLeft  = manualSpeed;
    targetRight = manualSpeed;
    currentState = STATE_MANUAL;
  } else if (cmd == "L") {
    targetLeft  = -turnSpeed;
    targetRight = turnSpeed;
    currentState = STATE_MANUAL;
  } else if (cmd == "R") {
    targetLeft  = turnSpeed;
    targetRight = -turnSpeed;
    currentState = STATE_MANUAL;
  } else if (cmd == "FL") {
    targetLeft  = -manualSpeed / 2;
    targetRight = -manualSpeed;
    currentState = STATE_MANUAL;
  } else if (cmd == "FR") {
    targetLeft  = -manualSpeed;
    targetRight = -manualSpeed / 2;
    currentState = STATE_MANUAL;
  } else if (cmd == "BL") {
    targetLeft  = manualSpeed / 2;
    targetRight = manualSpeed;
    currentState = STATE_MANUAL;
  } else if (cmd == "BR") {
    targetLeft  = manualSpeed;
    targetRight = manualSpeed / 2;
    currentState = STATE_MANUAL;
  } else {
    targetLeft  = 0;
    targetRight = 0;
    currentState = STATE_IDLE;
  }

  server.send(200, "text/plain", "OK");
}

void handleSettings() {
  setCorsHeaders();
  if (server.hasArg("speed"))    manualSpeed = constrain(server.arg("speed").toInt(), 60, 255);
  if (server.hasArg("turn"))     turnSpeed = constrain(server.arg("turn").toInt(), 60, 255);
  if (server.hasArg("brake"))    autoBrakeEnabled = (server.arg("brake") == "1" || server.arg("brake") == "true");
  if (server.hasArg("watchdog")) watchdogMs = constrain(server.arg("watchdog").toInt(), 200, 3000);

  String json = "{\"status\":\"OK\",\"speed\":" + String(manualSpeed) +
                ",\"turnSpeed\":" + String(turnSpeed) +
                ",\"autoBrake\":" + String(autoBrakeEnabled ? "true" : "false") +
                ",\"watchdog\":" + String(watchdogMs) + "}";
  server.send(200, "application/json", json);
}

// Built-in Browser Cockpit (Mobile-Responsive HTML5 HUD)
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>ESP32 Safety Rover HUD</title>
  <style>
    :root { --bg: #090d16; --card-bg: #131c2e; --border: #1e293b; --primary: #00f0ff; --danger: #ef4444; --success: #10b981; }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: system-ui, -apple-system, sans-serif; user-select: none; -webkit-user-select: none; }
    body { background: var(--bg); color: #e2e8f0; display: flex; flex-direction: column; align-items: center; min-height: 100vh; padding: 16px; touch-action: pan-y; }
    .header { width: 100%; max-width: 420px; display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
    .title { font-size: 1.1rem; font-weight: 700; color: var(--primary); letter-spacing: 1px; }
    .status-bar { width: 100%; max-width: 420px; padding: 12px; border-radius: 12px; text-align: center; font-weight: 700; font-size: 0.9rem; margin-bottom: 16px; background: var(--card-bg); border: 1px solid var(--border); transition: all 0.2s; }
    .status-bar.alert { background: rgba(239, 68, 68, 0.2); border-color: var(--danger); color: #fca5a5; }
    .status-bar.warn { background: rgba(255, 140, 0, 0.2); border-color: #ff8c00; color: #ffbe76; }
    .status-bar.active { background: rgba(0, 240, 255, 0.15); border-color: var(--primary); color: var(--primary); }
    .telemetry { display: flex; gap: 8px; width: 100%; max-width: 420px; margin-bottom: 18px; }
    .card { flex: 1; background: var(--card-bg); border: 1px solid var(--border); padding: 12px 6px; border-radius: 12px; text-align: center; }
    .card-label { font-size: 0.68rem; color: #94a3b8; font-weight: 600; text-transform: uppercase; }
    .val { font-size: 1.4rem; font-weight: 800; color: var(--primary); margin-top: 4px; }
    .val.danger { color: var(--danger); }
    .grid { display: grid; grid-template-columns: repeat(3, 85px); grid-template-rows: repeat(3, 85px); gap: 14px; justify-content: center; margin-bottom: 16px; }
    .btn { background: #1e293b; border: 1px solid #334155; border-radius: 18px; color: white; font-size: 1.8rem; display: flex; align-items: center; justify-content: center; cursor: pointer; touch-action: none; -webkit-tap-highlight-color: transparent; }
    .btn:active, .btn.pressed { background: var(--primary); color: #000; }
    .btn-stop { background: #991b1b; border-color: var(--danger); font-size: 0.95rem; font-weight: 800; }
    .btn-disabled { opacity: 0.3 !important; pointer-events: none !important; }
    .action-bar { width: 100%; max-width: 420px; display: flex; gap: 12px; justify-content: center; }
    .btn-arm { flex: 1; background: #065f46; border: 1px solid var(--success); color: white; border-radius: 12px; padding: 14px; font-weight: 800; font-size: 1rem; cursor: pointer; touch-action: manipulation; -webkit-tap-highlight-color: transparent; }
    .btn-arm:active { background: var(--success); color: #000; }
  </style>
</head>
<body>
  <div class="header">
    <div class="title">ROVER COCKPIT</div>
    <div id="netInfo" style="font-size:0.75rem; color:#64748b; margin-top:4px; letter-spacing:0.5px;">CONNECTING...</div>
  </div>
  <div id="status" class="status-bar active">SYSTEM ARMED &bull; READY</div>
  <div class="telemetry">
    <div class="card"><div class="card-label">Front Sonar</div><div id="fDist" class="val">-- cm</div></div>
    <div class="card"><div class="card-label">Alcohol Sensor</div><div id="alcVal" class="val">--</div></div>
    <div class="card"><div class="card-label">Left Sonar</div><div id="lDist" class="val">-- cm</div></div>
  </div>
  <div class="grid">
    <div></div>
    <button class="btn" id="btnF">&#9650;</button>
    <div></div>
    <button class="btn" id="btnL">&#9664;</button>
    <button class="btn btn-stop" id="btnStop">STOP</button>
    <button class="btn" id="btnR">&#9654;</button>
    <div></div>
    <button class="btn" id="btnB">&#9660;</button>
    <div></div>
  </div>
  <div class="action-bar">
    <button class="btn-arm" id="armBtn" onclick="manualArm()">RESTORE / RE-ARM</button>
  </div>
  <script>
    let activeCmd = 'S', hb = null, isBlocked = false, armCooldownUntil = 0;
    
    let fetchCtrl = new AbortController();
    async function sendRaw(val) {
      fetchCtrl.abort();
      fetchCtrl = new AbortController();
      try {
        await fetch('/cmd?val=' + val, { cache: 'no-store', signal: fetchCtrl.signal });
      } catch(e) {}
    }
    
    function startDrive(dir) {
      if (isBlocked) return;
      activeCmd = dir;
      sendRaw(dir);
      if (hb) clearInterval(hb);
      hb = setInterval(() => {
        if (activeCmd !== 'S' && !isBlocked) sendRaw(activeCmd);
      }, 140);
    }
    
    function stopDrive() {
      if (hb) { clearInterval(hb); hb = null; }
      activeCmd = 'S';
      sendRaw('S');
    }

    function bindDirBtn(id, cmd) {
      const el = document.getElementById(id);
      const start = (e) => { e.preventDefault(); startDrive(cmd); };
      const stop = (e) => { e.preventDefault(); stopDrive(); };
      el.addEventListener('pointerdown', start, { passive: false });
      el.addEventListener('pointerup', stop, { passive: false });
      el.addEventListener('pointercancel', stop, { passive: false });
      el.addEventListener('contextmenu', e => e.preventDefault());
    }

    bindDirBtn('btnF', 'F');
    bindDirBtn('btnB', 'B');
    bindDirBtn('btnL', 'L');
    bindDirBtn('btnR', 'R');

    document.getElementById('btnStop').addEventListener('click', (e) => {
      e.preventDefault();
      stopDrive();
    });

    async function manualArm() {
      armCooldownUntil = Date.now() + 2500;
      isBlocked = false;
      
      const s = document.getElementById('status');
      s.className = 'status-bar active';
      s.innerText = 'SYSTEM ARMED &bull; READY';
      
      document.querySelectorAll('.grid .btn').forEach(b => b.classList.remove('btn-disabled'));
      await sendRaw('ARM'); 
    }

    setInterval(() => {
      fetch('/status', { cache: 'no-store' }).then(r => r.json()).then(d => {
        if (d.ip) {
          const netEl = document.getElementById('netInfo');
          if (netEl) netEl.innerText = (d.wifiMode || 'STA') + ' \u2022 ' + d.ip + ' \u2022 http://esp32-rover.local';
        }
        document.getElementById('fDist').innerText = d.front + ' cm';
        document.getElementById('lDist').innerText = d.left + ' cm';
        
        const alcElem = document.getElementById('alcVal');
        alcElem.innerText = d.alcohol;
        
        const s = document.getElementById('status');
        const armBtn = document.getElementById('armBtn');

        if (d.alcoholDetected) {
          isBlocked = true;
          alcElem.className = 'val danger';
          s.className = 'status-bar alert';
          s.innerText = 'ALCOHOL DETECTED: IGNITION LOCKED';
          document.querySelectorAll('.grid .btn:not(.btn-stop)').forEach(b => b.classList.add('btn-disabled'));
          armBtn.classList.add('btn-disabled');
        } else if (d.state === 'PARKING') {
          isBlocked = true;
          alcElem.className = 'val';
          s.className = 'status-bar warn';
          s.innerText = 'MICROSLEEP: AUTO-PARKING TO LEFT CURB...';
          document.querySelectorAll('.grid .btn:not(.btn-stop)').forEach(b => b.classList.add('btn-disabled'));
          armBtn.classList.add('btn-disabled');
        } else if (d.state === 'HALTED') {
          if (Date.now() > armCooldownUntil) {
            isBlocked = true;
            alcElem.className = 'val';
            s.className = 'status-bar alert';
            s.innerText = 'PARKED & LOCKED: CLICK RESTORE/RE-ARM';
            document.querySelectorAll('.grid .btn:not(.btn-stop)').forEach(b => b.classList.add('btn-disabled'));
            armBtn.classList.remove('btn-disabled');
          }
        } else {
          isBlocked = false;
          alcElem.className = 'val';
          s.className = 'status-bar active';
          s.innerText = d.state === 'MANUAL' ? 'MANUAL DRIVE' : 'ARMED &bull; READY';
          document.querySelectorAll('.grid .btn').forEach(b => b.classList.remove('btn-disabled'));
          armBtn.classList.remove('btn-disabled');
        }
      }).catch(() => {});
    }, 250);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

// ======================== SETUP & LOOP ===================================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n==================================================");
  Serial.println("   ESP32 Autonomous Safety Rover Firmware Init   ");
  Serial.println("==================================================");

  pinMode(L_REN, OUTPUT); pinMode(L_LEN, OUTPUT);
  pinMode(R_REN, OUTPUT); pinMode(R_LEN, OUTPUT);

  digitalWrite(L_REN, LOW); digitalWrite(L_LEN, LOW);
  digitalWrite(R_REN, LOW); digitalWrite(R_LEN, LOW);

  initPwmPin(L_RPWM, L_RPWM_CH);
  initPwmPin(L_LPWM, L_LPWM_CH);
  initPwmPin(R_RPWM, R_RPWM_CH);
  initPwmPin(R_LPWM, R_LPWM_CH);

  // Initialize Primary Strip (20 LEDs on Pin 5)
  leds.begin();
  leds.setBrightness(LED_BRIGHTNESS);

  // Initialize Accent Strip (8 LEDs on Pin 14)
  leds2.begin();
  leds2.setBrightness(LED_BRIGHTNESS);
  leds2.clear();
  leds2.show();

  disconnectTyres();

  performAlcoholPreCheck();

  // ======================== COMMON WI-FI NETWORK INITIALIZATION =============
  WiFi.mode(WIFI_STA); 
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  Serial.println("\n--------------------------------------------------");
  Serial.print("[WIFI] Connecting to common network SSID: ");
  Serial.println(ssid);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 25) {
    delay(400);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected to Common Wi-Fi successfully!");
    Serial.print("[WIFI] Rover IP Address : http://");
    Serial.println(WiFi.localIP()); 
    Serial.print("[WIFI] Gateway          : ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("[WIFI] Signal Strength  : ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    if (MDNS.begin("esp32-rover")) {
      MDNS.addService("http", "tcp", 80);
      Serial.println("[mDNS] Responder active at: http://esp32-rover.local");
    } else {
      Serial.println("[mDNS] Error setting up mDNS responder");
    }
  } else {
    Serial.println("\n[WIFI] Warning: Could not connect to Station network!");
    Serial.println("[WIFI] Launching Fallback SoftAP to prevent system lockout...");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("ESP32-Safety-Car", "12345678");
    Serial.print("[WIFI] Emergency AP Active: SSID 'ESP32-Safety-Car' | IP: http://");
    Serial.println(WiFi.softAPIP());
    if (MDNS.begin("esp32-rover")) {
      MDNS.addService("http", "tcp", 80);
    }
  }

  udpServer.begin(UDP_PORT);
  Serial.print("[UDP] Auto-discovery listener active on port ");
  Serial.println(UDP_PORT);
  Serial.println("--------------------------------------------------\n"); 

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCommand);
  server.on("/cmd", HTTP_POST, handleCommand);
  server.on("/cmd", HTTP_OPTIONS, handleOptions);
  server.on("/drive", HTTP_GET, handleDrive);
  server.on("/drive", HTTP_POST, handleDrive);
  server.on("/drive", HTTP_OPTIONS, handleOptions);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/status", HTTP_OPTIONS, handleOptions);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/settings", HTTP_POST, handleSettings);
  server.on("/settings", HTTP_OPTIONS, handleOptions);
  server.on("/ping", HTTP_GET, handlePing);
  server.on("/ping", HTTP_OPTIONS, handleOptions);

  server.begin();
}

void loop() {
  server.handleClient();
  updateLEDs();

  unsigned long currentMillis = millis();

  // 1. Live Alcohol Polling - Noise Filtered
  if (currentMillis - lastAlcoholTick > 100) {
    lastAlcoholTick = currentMillis;
    
    int rawAlc = analogRead(MQ3_PIN);
    
    if (currentMillis < ignoreAlcoholUntil) {
        currentAlcoholVal = 0; 
        alcoholStrikeCount = 0;
    } else {
        currentAlcoholVal = (currentAlcoholVal * 2 + rawAlc) / 3;
        
        if (rawAlc >= ALCOHOL_THRESH) {
            alcoholStrikeCount++;
        } else {
            alcoholStrikeCount = 0;
        }
    }

    if (alcoholStrikeCount >= 4) {
      if (!alcoholLock) {
        alcoholLock = true;
        disconnectTyres();
        Serial.print(">>> [ALARM] Sustained Alcohol Detected! ADC: ");
        Serial.println(currentAlcoholVal);
      }
    } 
    else if (alcoholStrikeCount == 0 && alcoholLock) {
      alcoholLock = false;
      disconnectTyres();
      if (currentState != STATE_PARKING) {
        currentState = STATE_IDLE;
      }
      Serial.println(">>> [OK] Air Cleared. System Auto-Restored.");
    }
  }

  // 2. Serial Diagnostics Monitor
  if (currentMillis - lastSerialDebug > 500) {
    lastSerialDebug = currentMillis;
    Serial.print("[MQ-3] ADC: ");
    Serial.print(currentAlcoholVal);
    Serial.print(" / ");
    Serial.print(ALCOHOL_THRESH);
    Serial.print(" | Lock: ");
    Serial.print(alcoholLock ? "LOCKED" : "CLEAR");
    Serial.print(" | State: ");
    if (currentState == STATE_IDLE) Serial.println("IDLE");
    else if (currentState == STATE_MANUAL) Serial.println("MANUAL");
    else if (currentState == STATE_PARKING) Serial.println("PARKING");
    else Serial.println("HALTED");
  }

  // 3. 3-Phase Perfect Auto-Parking Execution Engine (Left Arc into Curb)
  if (currentState == STATE_PARKING) {
    unsigned long elapsed = currentMillis - parkStartMs;

    if (frontDist <= obstacleBrakeDist || leftDist <= PARK_CURB_CM) {
      finishAutoPark();
    }
    else if (elapsed < PARK_PHASE1_MS) {
      targetLeft   = -PARK_SPEED;
      targetRight  = -PARK_SPEED / 4;
    }
    else if (elapsed < (PARK_PHASE1_MS + PARK_PHASE2_MS)) {
      targetLeft   = -PARK_SPEED * 0.6;
      targetRight  = -PARK_SPEED * 0.6;
    }
    else if (elapsed < (PARK_PHASE1_MS + PARK_PHASE2_MS + PARK_PHASE3_MS)) {
      targetLeft   = 0;
      targetRight  = 0;
    }
    else {
      finishAutoPark();
    }
  }

  // 4. Motor Acceleration Ramp 
  if (currentMillis - lastRampTick > 15) {
    lastRampTick = currentMillis;
    if (alcoholLock || currentState == STATE_HALTED) {
      disconnectTyres();
    } else {
      updateMotorRamp();
    }
  }

  // 5. Watchdog for Manual Drive Mode
  if (currentState == STATE_MANUAL && (currentMillis - lastHeartbeat > watchdogMs)) {
    targetLeft   = 0;
    targetRight  = 0;
    currentState = STATE_IDLE;
  }

  // 6. Companion App Serial Reader 
  if (currentMillis - lastSensorTick > 80) {
    lastSensorTick = currentMillis;

    if (Serial.available() > 0) {
      String msg = Serial.readStringUntil('\n');
      msg.trim();
      msg.toUpperCase();
      if (msg == "STOP" || msg == "SLEEP" || msg == "PARK") {
        startAutoPark();
      } else if (msg == "ARM" || msg == "RESET" || msg == "RESTORE" || 
                 msg == "ENABLE" || msg == "RE-ENABLE" || msg == "REARM" || 
                 msg == "RE-ARM" || msg == "START" || msg == "RESUME") {
        if (!alcoholLock) {
          ignoreAlcoholUntil = millis() + 2500;
          currentState = STATE_IDLE;
          armMotorsHardware();
          lastHeartbeat = millis();
          Serial.println("[ARMED]");
        } 
      }
    }
  }

  // 7. Process UDP Discovery Requests
  int packetSize = udpServer.parsePacket();
  if (packetSize) {
    char packetBuffer[64];
    int len = udpServer.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) packetBuffer[len] = 0;
    if (strstr(packetBuffer, "DISCOVER_ROVER")) {
      IPAddress myIp = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : WiFi.softAPIP();
      String reply = "ROVER_HERE:http://" + myIp.toString();
      udpServer.beginPacket(udpServer.remoteIP(), udpServer.remotePort());
      udpServer.print(reply);
      udpServer.endPacket();
    }
  }
}