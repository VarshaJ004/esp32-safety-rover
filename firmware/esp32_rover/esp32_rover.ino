
/*
 * ==============================================================================
 * ESP32 Autonomous & Manual Safety Rover Controller Firmware
 * ==============================================================================
 * Target Hardware: ESP32 Development Board (NodeMCU / DevKit v1)
 * Motor Driver:    Dual BTS7960 43A H-Bridge Drivers
 * Sensors:         HC-SR04 / JSN-SR04T Ultrasonic Sensors (Front & Left)
 * Communication:   WiFi Access Point (192.168.4.1), HTTP REST API + WebSockets/CORS
 * Companion App:   Expo React Native Mobile App & Built-in Web Cockpit
 *
 * Supported Commands:
 *  - GET /cmd?val=F|B|L|R|FL|FR|BL|BR|S|STOP|PARK|SLEEP [&spd=0..255]
 *  - GET /drive?left=-255..255&right=-255..255 (Direct Proportional Steering)
 *  - GET /status -> Full JSON Telemetry (Distances, State, Battery, Motor levels)
 *  - GET /settings?speed=...&turn=...&brake=0|1&watchdog=...
 *  - GET /ping   -> Low-latency connection check
 * ==============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>

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

// --- PWM Configurations ---
#define PWM_FREQ        5000 // 5 kHz (silent motor switching)
#define PWM_RES         8    // 8-bit resolution (0 - 255)

// Core 2.x backward compatibility channels
#define L_RPWM_CH       0
#define L_LPWM_CH       1
#define R_RPWM_CH       2
#define R_LPWM_CH       3

// ======================== DEFAULT OPERATIONAL VALUES ======================
int manualSpeed         = 180;  // Normal driving speed (0 - 255)
int turnSpeed           = 155;  // Pivot turning speed (0 - 255)
unsigned long watchdogMs = 650;  // Auto-stop if no command received within window
bool autoBrakeEnabled   = true; // Prevent front collision if distance < obstacleBrakeDist
int obstacleBrakeDist   = 18;   // Minimum distance in cm to trigger auto-brake

// --- Access Point Credentials ---
const char *ssid        = "ESP32-Safety-Car";
const char *password    = "12345678";

// --- WebServer Instance (Port 80) ---
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

// Motor Ramp Targets for Smooth Motion (prevents high inrush currents)
int targetLeft   = 0;
int targetRight  = 0;
int currentLeft  = 0;
int currentRight = 0;

unsigned long lastSensorTick = 0;
unsigned long lastRampTick   = 0;
unsigned long lastHeartbeat  = 0;
bool alternateSensor         = false;

// ======================== HARDWARE PWM ABSTRACTIONS =======================
// Compatible with both ESP32 Arduino Core 2.x and Core 3.x
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

// Low-level hardware register write with gate isolation
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

// Complete physical tyre disconnect (Floats all MOSFET gates for safety)
void disconnectTyres() {
  targetLeft   = 0;
  targetRight  = 0;
  currentLeft  = 0;
  currentRight = 0;

  writePwm(L_RPWM, L_RPWM_CH, 0);
  writePwm(L_LPWM, L_LPWM_CH, 0);
  writePwm(R_RPWM, R_RPWM_CH, 0);
  writePwm(R_LPWM, R_LPWM_CH, 0);

  // Disabling the bridge lines cuts all current to the motors completely
  digitalWrite(L_REN, LOW);
  digitalWrite(L_LEN, LOW);
  digitalWrite(R_REN, LOW);
  digitalWrite(R_LEN, LOW);
}

// Software Acceleration Ramping (Smooth ramp prevents mechanical & electrical shock)
void updateMotorRamp() {
  const int RAMP_STEP = 25;

  if (currentLeft < targetLeft) currentLeft = min(currentLeft + RAMP_STEP, targetLeft);
  else if (currentLeft > targetLeft) currentLeft = max(currentLeft - RAMP_STEP, targetLeft);

  if (currentRight < targetRight) currentRight = min(currentRight + RAMP_STEP, targetRight);
  else if (currentRight > targetRight) currentRight = max(currentRight - RAMP_STEP, targetRight);

  applyMotors(currentLeft, currentRight);
}

// Fast ultrasonic read: 6ms timeout prevents CPU lock
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

// Ping / Heartbeat check
void handlePing() {
  setCorsHeaders();
  server.send(200, "application/json", "{\"pong\":true,\"uptime\":" + String(millis() / 1000) + "}");
}

// Telemetry endpoint for mobile app
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

// Differential Joystick Drive API: /drive?left=-255..255&right=-255..255
void handleDrive() {
  setCorsHeaders();
  lastHeartbeat = millis();

  if (currentState == STATE_HALTED) {
    disconnectTyres();
    server.send(403, "application/json", "{\"error\":\"Vehicle halted. Tap STOP to re-arm.\"}");
    return;
  }

  if (server.hasArg("left") && server.hasArg("right")) {
    int reqLeft  = server.arg("left").toInt();
    int reqRight = server.arg("right").toInt();

    reqLeft  = constrain(reqLeft, -255, 255);
    reqRight = constrain(reqRight, -255, 255);

    // Front obstacle auto-brake check
    if (autoBrakeEnabled && frontDist < obstacleBrakeDist && (reqLeft > 0 || reqRight > 0)) {
      targetLeft = 0;
      targetRight = 0;
      currentState = STATE_IDLE;
      server.send(200, "application/json", "{\"status\":\"BRAKED\",\"reason\":\"Front obstacle detected\"}");
      return;
    }

    targetLeft  = reqLeft;
    targetRight = reqRight;
    currentState = (targetLeft == 0 && targetRight == 0) ? STATE_IDLE : STATE_MANUAL;
    server.send(200, "application/json", "{\"status\":\"OK\",\"left\":" + String(targetLeft) + ",\"right\":" + String(targetRight) + "}");
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing left or right parameters\"}");
  }
}

// Directional Commands: /cmd?val=F|B|L|R|FL|FR|BL|BR|S|STOP [&spd=0..255]
void handleCommand() {
  setCorsHeaders();

  if (!server.hasArg("val")) {
    server.send(400, "text/plain", "Missing val");
    return;
  }

  String cmd = server.arg("val");
  lastHeartbeat = millis();

  // Allow dynamic speed override if provided
  if (server.hasArg("spd")) {
    int s = server.arg("spd").toInt();
    if (s >= 50 && s <= 255) manualSpeed = s;
  }

  // 1. MICROSLEEP / EMERGENCY STOP TRIGGER
  if (cmd == "STOP" || cmd == "PARK" || cmd == "SLEEP") {
    disconnectTyres();
    currentState = STATE_HALTED;
    Serial.println("[EMERGENCY] Microsleep triggered -> All tyres completely disconnected.");
    server.send(200, "text/plain", "Tyres Disconnected");
    return;
  }

  // 2. USER TAPS STOP BUTTON: Clears the lock and arms the vehicle
  if (cmd == "S") {
    disconnectTyres();
    currentState = STATE_IDLE;
    Serial.println("[RESET] Stop tapped -> Power restored, vehicle armed.");
    server.send(200, "text/plain", "System Armed");
    return;
  }

  // 3. HARD INTERLOCK: Block directional input if car is halted
  if (currentState == STATE_HALTED) {
    disconnectTyres();
    server.send(403, "text/plain", "Locked: Tap STOP to Re-Arm");
    return;
  }

  // 4. Auto-brake obstacle check for forward motion
  if (autoBrakeEnabled && frontDist < obstacleBrakeDist && (cmd == "F" || cmd == "FL" || cmd == "FR")) {
    disconnectTyres();
    currentState = STATE_IDLE;
    server.send(200, "text/plain", "Obstacle Auto-Brake");
    return;
  }

  // 5. Directional Maneuvers
  if (cmd == "F") {
    targetLeft  = manualSpeed;
    targetRight = manualSpeed;
    currentState = STATE_MANUAL;
  } else if (cmd == "B") {
    targetLeft  = -manualSpeed;
    targetRight = -manualSpeed;
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
    targetLeft  = manualSpeed / 2;
    targetRight = manualSpeed;
    currentState = STATE_MANUAL;
  } else if (cmd == "FR") {
    targetLeft  = manualSpeed;
    targetRight = manualSpeed / 2;
    currentState = STATE_MANUAL;
  } else if (cmd == "BL") {
    targetLeft  = -manualSpeed / 2;
    targetRight = -manualSpeed;
    currentState = STATE_MANUAL;
  } else if (cmd == "BR") {
    targetLeft  = -manualSpeed;
    targetRight = -manualSpeed / 2;
    currentState = STATE_MANUAL;
  } else {
    // Unrecognized command -> safe stop
    targetLeft  = 0;
    targetRight = 0;
    currentState = STATE_IDLE;
  }

  server.send(200, "text/plain", "OK");
}

// Runtime Settings Endpoint: /settings?speed=180&turn=155&brake=1&watchdog=650
void handleSettings() {
  setCorsHeaders();
  if (server.hasArg("speed")) {
    manualSpeed = constrain(server.arg("speed").toInt(), 60, 255);
  }
  if (server.hasArg("turn")) {
    turnSpeed = constrain(server.arg("turn").toInt(), 60, 255);
  }
  if (server.hasArg("brake")) {
    autoBrakeEnabled = (server.arg("brake") == "1" || server.arg("brake") == "true");
  }
  if (server.hasArg("watchdog")) {
    watchdogMs = constrain(server.arg("watchdog").toInt(), 200, 3000);
  }

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
    :root {
      --bg: #090d16;
      --card-bg: #131c2e;
      --border: #1e293b;
      --primary: #00f0ff;
      --danger: #ef4444;
      --success: #10b981;
      --warn: #f59e0b;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; user-select: none; -webkit-user-select: none; touch-action: manipulation; }
    body { background: var(--bg); color: #e2e8f0; display: flex; flex-direction: column; align-items: center; min-height: 100vh; padding: 16px; }
    .header { width: 100%; max-width: 420px; display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }
    .title { font-size: 1.1rem; font-weight: 700; color: var(--primary); letter-spacing: 1px; display: flex; align-items: center; gap: 8px; }
    .badge-dot { width: 10px; height: 10px; border-radius: 50%; background: var(--success); box-shadow: 0 0 10px var(--success); }
    .status-bar { width: 100%; max-width: 420px; padding: 12px; border-radius: 12px; text-align: center; font-weight: 700; font-size: 0.9rem; margin-bottom: 16px; background: var(--card-bg); border: 1px solid var(--border); transition: all 0.3s; }
    .status-bar.alert { background: rgba(239, 68, 68, 0.2); border-color: var(--danger); color: #fca5a5; animation: blink 1.2s infinite; }
    .status-bar.active { background: rgba(0, 240, 255, 0.15); border-color: var(--primary); color: var(--primary); }
    @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
    .telemetry { display: flex; gap: 12px; width: 100%; max-width: 420px; margin-bottom: 18px; }
    .card { flex: 1; background: var(--card-bg); border: 1px solid var(--border); padding: 14px; border-radius: 12px; text-align: center; }
    .card-label { font-size: 0.72rem; color: #94a3b8; font-weight: 600; text-transform: uppercase; letter-spacing: 0.5px; }
    .val { font-size: 1.6rem; font-weight: 800; color: var(--primary); margin-top: 4px; }
    .val.close { color: var(--danger) !important; }
    .grid { display: grid; grid-template-columns: repeat(3, 85px); grid-template-rows: repeat(3, 85px); gap: 14px; justify-content: center; margin-bottom: 20px; }
    .btn { background: #1e293b; border: 1px solid #334155; border-radius: 18px; color: white; font-size: 1.8rem; display: flex; align-items: center; justify-content: center; cursor: pointer; transition: all 0.08s; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
    .btn:active, .btn.pressed { background: var(--primary); color: #000; transform: scale(0.95); box-shadow: 0 0 20px rgba(0,240,255,0.6); }
    .btn-stop { background: #991b1b; border-color: var(--danger); font-size: 0.95rem; font-weight: 800; color: #fff; }
    .btn-stop:active { background: var(--danger); }
    .btn-disabled { opacity: 0.25; pointer-events: none; }
    .speed-slider { width: 100%; max-width: 420px; background: var(--card-bg); border: 1px solid var(--border); border-radius: 12px; padding: 14px; margin-bottom: 18px; }
    .slider-header { display: flex; justify-content: space-between; font-size: 0.8rem; font-weight: 600; color: #94a3b8; margin-bottom: 8px; }
    input[type=range] { width: 100%; accent-color: var(--primary); cursor: pointer; }
    .app-link { margin-top: auto; font-size: 0.8rem; color: #64748b; text-align: center; }
  </style>
</head>
<body>
  <div class="header">
    <div class="title"><div class="badge-dot" id="onlineDot"></div>ROVER COCKPIT</div>
    <div style="font-size:0.75rem; color:#64748b;">ESP32 v2.0</div>
  </div>

  <div id="status" class="status-bar active">SYSTEM ARMED &bull; READY</div>

  <div class="telemetry">
    <div class="card">
      <div class="card-label">Front Sonar</div>
      <div id="fDist" class="val">-- cm</div>
    </div>
    <div class="card">
      <div class="card-label">Left Sonar</div>
      <div id="lDist" class="val">-- cm</div>
    </div>
  </div>

  <div class="speed-slider">
    <div class="slider-header">
      <span>THROTTLE PWM</span>
      <span id="speedVal" style="color:var(--primary);">180</span>
    </div>
    <input type="range" min="100" max="255" value="180" id="spdInput" onchange="updateSpeed(this.value)">
  </div>

  <div class="grid" id="dpad">
    <div></div>
    <button class="btn" id="btnF" onpointerdown="startDrive(event, 'F')" onpointerup="stopDrive(event)">&#9650;</button>
    <div></div>
    <button class="btn" id="btnL" onpointerdown="startDrive(event, 'L')" onpointerup="stopDrive(event)">&#9664;</button>
    <button class="btn btn-stop" onclick="resetPower(event)">STOP</button>
    <button class="btn" id="btnR" onpointerdown="startDrive(event, 'R')" onpointerup="stopDrive(event)">&#9654;</button>
    <div></div>
    <button class="btn" id="btnB" onpointerdown="startDrive(event, 'B')" onpointerup="stopDrive(event)">&#9660;</button>
    <div></div>
  </div>

  <div class="app-link">Expo Mobile App Companion Mode Supported</div>

  <script>
    let activeCmd = 'S';
    let heartbeatTimer = null;
    let isHalted = false;
    let currentPwm = 180;

    function sendCmd(val) {
      fetch('/cmd?val=' + val + '&spd=' + currentPwm, { cache: 'no-store' }).catch(() => {});
    }

    function updateSpeed(v) {
      currentPwm = parseInt(v);
      document.getElementById('speedVal').innerText = currentPwm;
      fetch('/settings?speed=' + currentPwm).catch(() => {});
    }

    function startDrive(e, dir) {
      if (isHalted) return;
      if (e) {
        e.preventDefault();
        try { e.target.setPointerCapture(e.pointerId); } catch(err) {}
        e.target.classList.add('pressed');
      }
      activeCmd = dir;
      sendCmd(dir);
      if (heartbeatTimer) clearInterval(heartbeatTimer);
      heartbeatTimer = setInterval(() => { if (activeCmd !== 'S') sendCmd(activeCmd); }, 160);
    }

    function stopDrive(e) {
      if (e) {
        e.preventDefault();
        try { e.target.releasePointerCapture(e.pointerId); } catch(err) {}
        e.target.classList.remove('pressed');
      }
      if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
      if (!isHalted) {
        activeCmd = 'S';
        sendCmd('S');
      }
    }

    function resetPower(e) {
      if (e) e.preventDefault();
      if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
      activeCmd = 'S';
      sendCmd('S');
    }

    // Keyboard bindings for testing
    let activeKey = null;
    window.addEventListener('keydown', e => {
      if (activeKey === e.key) return;
      activeKey = e.key;
      if (e.key === 'ArrowUp' || e.key === 'w') startDrive(null, 'F');
      else if (e.key === 'ArrowDown' || e.key === 's') startDrive(null, 'B');
      else if (e.key === 'ArrowLeft' || e.key === 'a') startDrive(null, 'L');
      else if (e.key === 'ArrowRight' || e.key === 'd') startDrive(null, 'R');
      else if (e.key === ' ') resetPower(null);
    });

    window.addEventListener('keyup', e => {
      activeKey = null;
      if (e.key !== ' ') stopDrive(null);
    });

    // Telemetry polling
    setInterval(() => {
      fetch('/status', { cache: 'no-store' })
        .then(r => r.json())
        .then(d => {
          const fElem = document.getElementById('fDist');
          const lElem = document.getElementById('lDist');
          fElem.innerText = d.front + ' cm';
          lElem.innerText = d.left + ' cm';

          fElem.className = 'val ' + (d.front < 20 ? 'close' : '');
          lElem.className = 'val ' + (d.left < 20 ? 'close' : '');

          const s = document.getElementById('status');
          if (d.state === 'HALTED') {
            isHalted = true;
            s.className = 'status-bar alert';
            s.innerText = 'MICROSLEEP DETECTED: TYRES DISCONNECTED (TAP STOP TO RE-ARM)';
            document.querySelectorAll('.btn:not(.btn-stop)').forEach(b => b.classList.add('btn-disabled'));
          } else {
            isHalted = false;
            s.className = 'status-bar active';
            s.innerText = d.state === 'MANUAL' ? 'MANUAL DRIVE ACTIVE' : 'POWER ARMED &bull; READY';
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

  // Initialize motor bridge pins LOW
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

  // Ultrasonic sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  // Initialize PWM channels
  initPwmPin(L_RPWM, L_RPWM_CH);
  initPwmPin(L_LPWM, L_LPWM_CH);
  initPwmPin(R_RPWM, R_RPWM_CH);
  initPwmPin(R_LPWM, R_LPWM_CH);

  // Ensure motors start completely isolated
  disconnectTyres();

  // Configure WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.setSleep(false); // Disable WiFi power saving for ultra-low latency
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  Serial.println("\n=======================================================");
  Serial.println("  ESP32 Autonomous & Manual Safety Rover Online");
  Serial.println("=======================================================");
  Serial.print("  [WIFI] Hotspot SSID:  "); Serial.println(ssid);
  Serial.print("  [WIFI] Hotspot IP:    http://"); Serial.println(WiFi.softAPIP());
  Serial.println("=======================================================");

  // Setup Web Server Routes
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

  // 2. Safety Watchdog for Manual Driving (Auto-stops if phone goes out of range)
  if (currentState == STATE_MANUAL && (currentMillis - lastHeartbeat > watchdogMs)) {
    targetLeft   = 0;
    targetRight  = 0;
    currentState = STATE_IDLE;
  }

  // 3. Sensor Routine (Every 80ms, alternating between front and left)
  if (currentMillis - lastSensorTick > 80) {
    lastSensorTick = currentMillis;

    if (alternateSensor) {
      frontDist = readDistanceNonBlock(TRIG_FRONT, ECHO_FRONT);
      // Auto-brake check while driving forward
      if (autoBrakeEnabled && frontDist < obstacleBrakeDist && currentState == STATE_MANUAL) {
        if (targetLeft > 0 || targetRight > 0) {
          targetLeft   = 0;
          targetRight  = 0;
          currentState = STATE_IDLE;
          Serial.println("[AUTO-BRAKE] Front obstacle detected -> Stopped!");
        }
      }
    } else {
      leftDist = readDistanceNonBlock(TRIG_LEFT, ECHO_LEFT);
    }
    alternateSensor = !alternateSensor;

    // External Serial trigger fallback (for companion Raspberry Pi / PC microsleep detection)
    if (Serial.available() > 0) {
      String msg = Serial.readStringUntil('\n');
      msg.trim();
      if (msg == "STOP" || msg == "SLEEP" || msg == "PARK") {
        disconnectTyres();
        currentState = STATE_HALTED;
        Serial.println("[SERIAL] Microsleep signal received -> Tyres disconnected.");
      } else if (msg == "S") {
        disconnectTyres();
        currentState = STATE_IDLE;
        Serial.println("[SERIAL] Reset received -> Power re-armed.");
      }
    }
  }
}
