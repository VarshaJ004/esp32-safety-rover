/*
 * ==============================================================================
 * ESP32 Autonomous & Manual Safety Rover Controller Firmware
 * ==============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>
#include <Adafruit_NeoPixel.h>

// ======================== HARDWARE PIN DEFINITIONS ========================

// --- Left BTS7960 Driver Pins ---
#define L_RPWM          19   // Forward PWM
#define L_LPWM          18   // Reverse PWM
#define L_REN           21   // Forward Enable
#define L_LEN           22   // Reverse Enable

// --- Right BTS7960 Driver Pins ---
#define R_RPWM          25   // Forward PWM
#define R_LPWM          23   // Reverse PWM
#define R_REN           27   // Forward Enable
#define R_LEN           32   // Reverse Enable

// --- Ultrasonic Sensors ---
#define TRIG_FRONT      4
#define ECHO_FRONT      16
#define TRIG_LEFT       26
#define ECHO_LEFT       17

// --- WS2812B NeoPixel Setup ---
#define LED_PIN         5  // Change this to your WS2812 DIN GPIO pin
#define NUM_LEDS        10   // 10 LEDs total (0..4 Right, 5..9 Left)
#define LED_BRIGHTNESS  120  // 0 - 255

Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- PWM Configurations ---
#define PWM_FREQ        5000 // 5 kHz
#define PWM_RES         8    // 8-bit (0 - 255)

#define L_RPWM_CH       0
#define L_LPWM_CH       1
#define R_RPWM_CH       2
#define R_LPWM_CH       3

// ======================== DEFAULT OPERATIONAL VALUES ======================
int manualSpeed         = 180;
int turnSpeed           = 155;
unsigned long watchdogMs = 650;
bool autoBrakeEnabled   = true;
int obstacleBrakeDist   = 18;

const char *ssid        = "ESP32-Safety-Car";
const char *password    = "12345678";

WebServer server(80);

// ======================== STATE MACHINE & TELEMETRY =======================
enum CarState {
  STATE_IDLE,
  STATE_MANUAL,
  STATE_HALTED
};

volatile CarState currentState = STATE_IDLE;
volatile long frontDist        = 200;
volatile long leftDist         = 200;

int targetLeft   = 0;
int targetRight  = 0;
int currentLeft  = 0;
int currentRight = 0;

unsigned long lastSensorTick = 0;
unsigned long lastRampTick   = 0;
unsigned long lastHeartbeat  = 0;
bool alternateSensor         = false;

// ======================== LED CONTROL ROUTINES ===========================
unsigned long lastLedTick = 0;
int ledChasePosition = 0;

void ledsOff() {
  leds.clear();
  leds.show();
}

void ledsWhite() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds.setPixelColor(i, leds.Color(255, 255, 255));
  }
  leds.show();
}

void ledsLeftIndicator() {
  leds.clear();
  // LEDs 5 to 9 (Left Half)
  int head = 5 + ledChasePosition;
  int tail = 5 + ((ledChasePosition + 4) % 5);
  leds.setPixelColor(head, leds.Color(255, 90, 0));
  leds.setPixelColor(tail, leds.Color(100, 35, 0));
  leds.show();
}

void ledsRightIndicator() {
  leds.clear();
  // LEDs 0 to 4 (Right Half)
  int head = ledChasePosition;
  int tail = (ledChasePosition + 4) % 5;
  leds.setPixelColor(head, leds.Color(255, 90, 0));
  leds.setPixelColor(tail, leds.Color(100, 35, 0));
  leds.show();
}

void ledsMicrosleep() {
  leds.clear();
  // Red chasing effect across all 10 LEDs
  int head = ledChasePosition;
  int tail = (ledChasePosition + NUM_LEDS - 1) % NUM_LEDS;
  leds.setPixelColor(head, leds.Color(255, 0, 0));
  leds.setPixelColor(tail, leds.Color(80, 0, 0));
  leds.show();
}

void updateLEDs() {
  unsigned long now = millis();

  // 1. MICROSLEEP LOCK: All 10 LEDs Red Chaser
  if (currentState == STATE_HALTED) {
    if (now - lastLedTick >= 80) {
      lastLedTick = now;
      ledChasePosition = (ledChasePosition + 1) % NUM_LEDS;
      ledsMicrosleep();
    }
    return;
  }

  // 2. TURN DETECTION (Handles Pivot L/R, Forward Curves FL/FR, and Reverse Curves BL/BR)
  bool isTurningLeft = (targetLeft < 0 && targetRight > 0) ||                               // Pivot Left ('L')
                       (targetLeft < 0 && targetRight < 0 && targetRight < targetLeft) ||   // Forward curve Left ('FL')
                       (targetLeft > 0 && targetRight > 0 && targetRight > targetLeft);     // Reverse curve Left ('BL')

  bool isTurningRight = (targetLeft > 0 && targetRight < 0) ||                              // Pivot Right ('R')
                        (targetLeft < 0 && targetRight < 0 && targetLeft < targetRight) ||  // Forward curve Right ('FR')
                        (targetLeft > 0 && targetRight > 0 && targetLeft > targetRight);    // Reverse curve Right ('BR')

  // Left turn: ONLY LEDs 6-10 (indexes 5-9) orange chase
  if (currentState == STATE_MANUAL && isTurningLeft) {
    if (now - lastLedTick >= 90) {
      lastLedTick = now;
      ledChasePosition = (ledChasePosition + 1) % 5;
      ledsLeftIndicator();
    }
    return;
  }

  // Right turn: ONLY LEDs 1-5 (indexes 0-4) orange chase
  if (currentState == STATE_MANUAL && isTurningRight) {
    if (now - lastLedTick >= 90) {
      lastLedTick = now;
      ledChasePosition = (ledChasePosition + 1) % 5;
      ledsRightIndicator();
    }
    return;
  }

  // 3. NORMAL DRIVING / IDLE: Full solid white lights
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
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left Bridge
  if (leftSpeed == 0) {
    writePwm(L_RPWM, L_RPWM_CH, 0);
    writePwm(L_LPWM, L_LPWM_CH, 0);
    digitalWrite(L_REN, LOW);
    digitalWrite(L_LEN, LOW);
  } else {
    digitalWrite(L_REN, HIGH);
    digitalWrite(L_LEN, HIGH);
    if (leftSpeed > 0) {
      writePwm(L_RPWM, L_RPWM_CH, leftSpeed);
      writePwm(L_LPWM, L_LPWM_CH, 0);
    } else {
      writePwm(L_RPWM, L_RPWM_CH, 0);
      writePwm(L_LPWM, L_LPWM_CH, abs(leftSpeed));
    }
  }

  // Right Bridge
  if (rightSpeed == 0) {
    writePwm(R_RPWM, R_RPWM_CH, 0);
    writePwm(R_LPWM, R_LPWM_CH, 0);
    digitalWrite(R_REN, LOW);
    digitalWrite(R_LEN, LOW);
  } else {
    digitalWrite(R_REN, HIGH);
    digitalWrite(R_LEN, HIGH);
    if (rightSpeed > 0) {
      writePwm(R_RPWM, R_RPWM_CH, rightSpeed);
      writePwm(R_LPWM, R_LPWM_CH, 0);
    } else {
      writePwm(R_RPWM, R_RPWM_CH, 0);
      writePwm(R_LPWM, R_LPWM_CH, abs(rightSpeed));
    }
  }
}

void disconnectTyres() {
  targetLeft   = 0;
  targetRight  = 0;
  currentLeft  = 0;
  currentRight = 0;

  writePwm(L_RPWM, L_RPWM_CH, 0);
  writePwm(L_LPWM, L_LPWM_CH, 0);
  writePwm(R_RPWM, R_RPWM_CH, 0);
  writePwm(R_LPWM, R_LPWM_CH, 0);

  digitalWrite(L_REN, LOW);
  digitalWrite(L_LEN, LOW);
  digitalWrite(R_REN, LOW);
  digitalWrite(R_LEN, LOW);
}

void updateMotorRamp() {
  const int RAMP_STEP = 25;

  if (currentLeft < targetLeft) currentLeft = min(currentLeft + RAMP_STEP, targetLeft);
  else if (currentLeft > targetLeft) currentLeft = max(currentLeft - RAMP_STEP, targetLeft);

  if (currentRight < targetRight) currentRight = min(currentRight + RAMP_STEP, targetRight);
  else if (currentRight > targetRight) currentRight = max(currentRight - RAMP_STEP, targetRight);

  applyMotors(currentLeft, currentRight);
}

long readDistanceNonBlock(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 6000);
  if (duration == 0) return 150;
  long dist = duration * 0.034 / 2;
  if (dist <= 0 || dist > 400) return 150;
  return dist;
}

// ======================== HTTP API & CORS HELPERS =========================
void setCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, X-Requested-With, Cache-Control");
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
}

void handleOptions() {
  setCorsHeaders();
  server.send(204);
}

void handlePing() {
  setCorsHeaders();
  server.send(200, "application/json", "{\"pong\":true,\"uptime\":" + String(millis() / 1000) + "}");
}

void handleStatus() {
  setCorsHeaders();
  String st = "IDLE";
  if (currentState == STATE_HALTED) st = "HALTED";
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
  json += "\"speed\":" + String(manualSpeed) + ",";
  json += "\"turnSpeed\":" + String(turnSpeed) + ",";
  json += "\"targetLeft\":" + String(targetLeft) + ",";
  json += "\"targetRight\":" + String(targetRight) + ",";
  json += "\"currentLeft\":" + String(currentLeft) + ",";
  json += "\"currentRight\":" + String(currentRight) + ",";
  json += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
  json += "\"uptime\":" + String(millis() / 1000) + ",";
  json += "\"watchdog\":" + String(remainingWatchdog) + ",";
  json += "\"autoBrake\":" + String(autoBrakeEnabled ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

void handleDrive() {
  setCorsHeaders();
  lastHeartbeat = millis();

  if (currentState == STATE_HALTED) {
    disconnectTyres();
    server.send(403, "application/json", "{\"error\":\"Vehicle halted. Re-arm required.\"}");
    return;
  }

  if (server.hasArg("left") && server.hasArg("right")) {
    int reqLeft  = -server.arg("left").toInt();
    int reqRight = -server.arg("right").toInt();

    reqLeft  = constrain(reqLeft, -255, 255);
    reqRight = constrain(reqRight, -255, 255);

    if (autoBrakeEnabled && frontDist < obstacleBrakeDist && (reqLeft < 0 || reqRight < 0)) {
      targetLeft = 0;
      targetRight = 0;
      currentState = STATE_IDLE;
      server.send(200, "application/json", "{\"status\":\"BRAKED\",\"reason\":\"Front obstacle detected\"}");
      return;
    }

    targetLeft   = reqLeft;
    targetRight  = reqRight;
    currentState = (targetLeft == 0 && targetRight == 0) ? STATE_IDLE : STATE_MANUAL;
    server.send(200, "application/json", "{\"status\":\"OK\",\"left\":" + String(targetLeft) + ",\"right\":" + String(targetRight) + "}");
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing left or right parameters\"}");
  }
}

void handleCommand() {
  setCorsHeaders();

  if (!server.hasArg("val")) {
    server.send(400, "text/plain", "Missing val");
    return;
  }

  String cmd = server.arg("val");
  lastHeartbeat = millis();

  if (server.hasArg("spd")) {
    int s = server.arg("spd").toInt();
    if (s >= 50 && s <= 255) manualSpeed = s;
  }

  // Python app / UI triggers Microsleep Stop
  if (cmd == "STOP" || cmd == "PARK" || cmd == "SLEEP") {
    disconnectTyres();
    currentState = STATE_HALTED;
    server.send(200, "text/plain", "Tyres Disconnected");
    return;
  }

  // Python app / UI resets lock when eyes reopen
  if (cmd == "S" || cmd == "RESET" || cmd == "ARM") {
    disconnectTyres();
    currentState = STATE_IDLE;
    server.send(200, "text/plain", "System Armed");
    return;
  }

  if (currentState == STATE_HALTED) {
    disconnectTyres();
    server.send(403, "text/plain", "Locked: Re-Arm Required");
    return;
  }

  if (autoBrakeEnabled && frontDist < obstacleBrakeDist && (cmd == "F" || cmd == "FL" || cmd == "FR")) {
    disconnectTyres();
    currentState = STATE_IDLE;
    server.send(200, "text/plain", "Obstacle Auto-Brake");
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
  if (server.hasArg("speed"))   manualSpeed = constrain(server.arg("speed").toInt(), 60, 255);
  if (server.hasArg("turn"))    turnSpeed = constrain(server.arg("turn").toInt(), 60, 255);
  if (server.hasArg("brake"))   autoBrakeEnabled = (server.arg("brake") == "1" || server.arg("brake") == "true");
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
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: system-ui, -apple-system, sans-serif; user-select: none; }
    body { background: var(--bg); color: #e2e8f0; display: flex; flex-direction: column; align-items: center; min-height: 100vh; padding: 16px; }
    .header { width: 100%; max-width: 420px; display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
    .title { font-size: 1.1rem; font-weight: 700; color: var(--primary); letter-spacing: 1px; }
    .status-bar { width: 100%; max-width: 420px; padding: 12px; border-radius: 12px; text-align: center; font-weight: 700; font-size: 0.9rem; margin-bottom: 16px; background: var(--card-bg); border: 1px solid var(--border); transition: all 0.3s; }
    .status-bar.alert { background: rgba(239, 68, 68, 0.2); border-color: var(--danger); color: #fca5a5; }
    .status-bar.active { background: rgba(0, 240, 255, 0.15); border-color: var(--primary); color: var(--primary); }
    .telemetry { display: flex; gap: 12px; width: 100%; max-width: 420px; margin-bottom: 18px; }
    .card { flex: 1; background: var(--card-bg); border: 1px solid var(--border); padding: 14px; border-radius: 12px; text-align: center; }
    .card-label { font-size: 0.72rem; color: #94a3b8; font-weight: 600; text-transform: uppercase; }
    .val { font-size: 1.6rem; font-weight: 800; color: var(--primary); margin-top: 4px; }
    .grid { display: grid; grid-template-columns: repeat(3, 85px); grid-template-rows: repeat(3, 85px); gap: 14px; justify-content: center; margin-bottom: 20px; }
    .btn { background: #1e293b; border: 1px solid #334155; border-radius: 18px; color: white; font-size: 1.8rem; display: flex; align-items: center; justify-content: center; cursor: pointer; }
    .btn:active, .btn.pressed { background: var(--primary); color: #000; }
    .btn-stop { background: #991b1b; border-color: var(--danger); font-size: 0.95rem; font-weight: 800; }
    .btn-disabled { opacity: 0.25; pointer-events: none; }
  </style>
</head>
<body>
  <div class="header"><div class="title">ROVER COCKPIT</div></div>
  <div id="status" class="status-bar active">SYSTEM ARMED &bull; READY</div>
  <div class="telemetry">
    <div class="card"><div class="card-label">Front Sonar</div><div id="fDist" class="val">-- cm</div></div>
    <div class="card"><div class="card-label">Left Sonar</div><div id="lDist" class="val">-- cm</div></div>
  </div>
  <div class="grid">
    <div></div>
    <button class="btn" onpointerdown="startDrive('F')" onpointerup="stopDrive()">&#9650;</button>
    <div></div>
    <button class="btn" onpointerdown="startDrive('L')" onpointerup="stopDrive()">&#9664;</button>
    <button class="btn btn-stop" onclick="resetPower()">STOP</button>
    <button class="btn" onpointerdown="startDrive('R')" onpointerup="stopDrive()">&#9654;</button>
    <div></div>
    <button class="btn" onpointerdown="startDrive('B')" onpointerup="stopDrive()">&#9660;</button>
    <div></div>
  </div>
  <script>
    let activeCmd = 'S', hb = null, isHalted = false;
    function sendCmd(val) { fetch('/cmd?val=' + val, { cache: 'no-store' }).catch(() => {}); }
    function startDrive(dir) {
      if (isHalted) return;
      activeCmd = dir; sendCmd(dir);
      if (hb) clearInterval(hb);
      hb = setInterval(() => { if (activeCmd !== 'S') sendCmd(activeCmd); }, 160);
    }
    function stopDrive() {
      if (hb) { clearInterval(hb); hb = null; }
      if (!isHalted) { activeCmd = 'S'; sendCmd('S'); }
    }
    function resetPower() {
      if (hb) { clearInterval(hb); hb = null; }
      activeCmd = 'S'; sendCmd('S');
    }
    setInterval(() => {
      fetch('/status', { cache: 'no-store' }).then(r => r.json()).then(d => {
        document.getElementById('fDist').innerText = d.front + ' cm';
        document.getElementById('lDist').innerText = d.left + ' cm';
        const s = document.getElementById('status');
        if (d.state === 'HALTED') {
          isHalted = true; s.className = 'status-bar alert';
          s.innerText = 'MICROSLEEP LOCK: TYRES DISCONNECTED';
          document.querySelectorAll('.btn:not(.btn-stop)').forEach(b => b.classList.add('btn-disabled'));
        } else {
          isHalted = false; s.className = 'status-bar active';
          s.innerText = d.state === 'MANUAL' ? 'MANUAL DRIVE' : 'ARMED &bull; READY';
          document.querySelectorAll('.btn').forEach(b => b.classList.remove('btn-disabled'));
        }
      }).catch(() => {});
    }, 250);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// ======================== ARDUINO SETUP ===================================
void setup() {
  Serial.begin(115200);

  digitalWrite(L_RPWM, LOW);
  digitalWrite(L_LPWM, LOW);
  digitalWrite(L_REN, LOW);
  digitalWrite(L_LEN, LOW);
  digitalWrite(R_RPWM, LOW);
  digitalWrite(R_LPWM, LOW);
  digitalWrite(R_REN, LOW);
  digitalWrite(R_LEN, LOW);

  pinMode(L_REN, OUTPUT);
  pinMode(L_LEN, OUTPUT);
  pinMode(R_REN, OUTPUT);
  pinMode(R_LEN, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  initPwmPin(L_RPWM, L_RPWM_CH);
  initPwmPin(L_LPWM, L_LPWM_CH);
  initPwmPin(R_RPWM, R_RPWM_CH);
  initPwmPin(R_LPWM, R_LPWM_CH);

  leds.begin();
  leds.setBrightness(LED_BRIGHTNESS);
  ledsWhite();

  disconnectTyres();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCommand);
  server.on("/cmd", HTTP_OPTIONS, handleOptions);
  server.on("/drive", HTTP_GET, handleDrive);
  server.on("/drive", HTTP_OPTIONS, handleOptions);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/status", HTTP_OPTIONS, handleOptions);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/settings", HTTP_OPTIONS, handleOptions);
  server.on("/ping", HTTP_GET, handlePing);
  server.on("/ping", HTTP_OPTIONS, handleOptions);

  server.begin();
}

// ======================== MAIN CONTROL LOOP ==============================
void loop() {
  server.handleClient();

  // Run non-blocking LED animation engine
  updateLEDs();

  unsigned long currentMillis = millis();

  // 1. Acceleration Ramp Engine (Runs every 15ms)
  if (currentMillis - lastRampTick > 15) {
    lastRampTick = currentMillis;
    if (currentState == STATE_HALTED) {
      disconnectTyres();
    } else {
      updateMotorRamp();
    }
  }

  // 2. Safety Watchdog for Manual Driving
  if (currentState == STATE_MANUAL && (currentMillis - lastHeartbeat > watchdogMs)) {
    targetLeft   = 0;
    targetRight  = 0;
    currentState = STATE_IDLE;
  }

  // 3. Sensor Routine (Every 80ms)
  if (currentMillis - lastSensorTick > 80) {
    lastSensorTick = currentMillis;

    if (alternateSensor) {
      frontDist = readDistanceNonBlock(TRIG_FRONT, ECHO_FRONT);
      if (autoBrakeEnabled && frontDist < obstacleBrakeDist && currentState == STATE_MANUAL) {
        if (targetLeft < 0 || targetRight < 0) {
          targetLeft   = 0;
          targetRight  = 0;
          currentState = STATE_IDLE;
        }
      }
    } else {
      leftDist = readDistanceNonBlock(TRIG_LEFT, ECHO_LEFT);
    }
    alternateSensor = !alternateSensor;

    // 4. Python Companion App Serial Reader (USB or Hardware UART)
    if (Serial.available() > 0) {
      String msg = Serial.readStringUntil('\n');
      msg.trim();
      if (msg == "STOP" || msg == "SLEEP" || msg == "PARK") {
        disconnectTyres();
        currentState = STATE_HALTED;
      } else if (msg == "S" || msg == "RESET" || msg == "ARM") {
        disconnectTyres();
        currentState = STATE_IDLE;
      }
    }
  }
}