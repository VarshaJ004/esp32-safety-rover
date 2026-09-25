# ESP32 Safety Rover Firmware & Hardware Integration Guide

This directory contains the upgraded, production-grade firmware for the ESP32-powered Safety Rover, designed to interface seamlessly with both the **Expo Mobile App** and the built-in fallback **HTML5 Web Cockpit**.

---

## 🛠 Hardware Architecture & Pin Mapping

### 1. Left BTS7960 43A Motor Driver
| Pin on BTS7960 | ESP32 GPIO | Description |
| :--- | :--- | :--- |
| **RPWM** | `GPIO 19` | Forward PWM Speed Control |
| **LPWM** | `GPIO 18` | Reverse PWM Speed Control |
| **R_EN** | `GPIO 21` | Forward Bridge Enable (HIGH = active) |
| **L_EN** | `GPIO 22` | Reverse Bridge Enable (HIGH = active) |
| **VCC** | `5V` (VIN) | Driver logic power |
| **GND** | `GND` | Common ground with ESP32 |

### 2. Right BTS7960 43A Motor Driver
| Pin on BTS7960 | ESP32 GPIO | Description |
| :--- | :--- | :--- |
| **RPWM** | `GPIO 25` | Forward PWM Speed Control |
| **LPWM** | `GPIO 23` | Reverse PWM Speed Control |
| **R_EN** | `GPIO 27` | Forward Bridge Enable (HIGH = active) |
| **L_EN** | `GPIO 32` | Reverse Bridge Enable (HIGH = active) |
| **VCC** | `5V` (VIN) | Driver logic power |
| **GND** | `GND` | Common ground with ESP32 |

### 3. Ultrasonic Distance Sensors
| Sensor | Trig Pin (ESP32) | Echo Pin (ESP32) | Recommended VCC | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Front HC-SR04** | `GPIO 4` | `GPIO 16` | 5V | Use 1k/2k resistor voltage divider on Echo pin to protect 3.3V GPIO |
| **Left HC-SR04** | `GPIO 26` | `GPIO 17` | 5V | Use 1k/2k resistor voltage divider on Echo pin |

> ⚠️ **CRITICAL NOTE ON GROUNDS**: All grounds (Battery (-), BTS7960 GND, ESP32 GND, and Sensor GND) **MUST** be tied together as a single common ground.

---

## ⚡ Motor Power Wiring

```text
[ LiPo / LiFePO4 Battery: 7.4V - 24V ]
       │                  │
      (+)                (-)
       ├──────────────┬───┴──────────────┐
       │              │                  │
 [ Left BTS7960 ] [ Right BTS7960 ] [ Buck Converter (5V 3A) ]
   B+     B-        B+     B-             IN+        IN-
   │      │         │      │               │          │
   M+     M-        M+     M-             OUT+ (5V)  OUT- (GND)
   │      │         │      │               │          │
[Left Motor]    [Right Motor]         [ ESP32 VIN & Drivers VCC ]
```

---

## 📡 REST API & Network Endpoints

The ESP32 broadcasts a Wi-Fi Access Point:
- **SSID**: `ESP32-Safety-Car`
- **Password**: `12345678`
- **Gateway IP**: `http://192.168.4.1`

All endpoints include CORS headers (`Access-Control-Allow-Origin: *`) and handle `OPTIONS` preflight requests so both React Native and Web environments can connect directly.

### 1. `GET /cmd?val=<COMMAND>&spd=<SPEED>`
Executes vehicle motion commands:
- `val=F`: Move Forward
- `val=B`: Move Backward
- `val=L`: Turn Left (Pivot)
- `val=R`: Turn Right (Pivot)
- `val=FL`: Forward-Left curve
- `val=FR`: Forward-Right curve
- `val=BL`: Backward-Left curve
- `val=BR`: Backward-Right curve
- `val=S`: Safe Stop (Clears emergency lock and restores motor drive)
- `val=STOP` or `val=SLEEP`: **EMERGENCY DISCONNECT**. Floats all MOSFET gates (R_EN / L_EN set to LOW). Controls stay locked until `S` is received.
- `spd` *(optional)*: Override PWM speed (range `50` to `255`).

### 2. `GET /drive?left=<PWM>&right=<PWM>`
Direct differential drive for virtual joysticks:
- `left`: Motor PWM from `-255` (full reverse) to `+255` (full forward).
- `right`: Motor PWM from `-255` to `+255`.

### 3. `GET /status`
Returns live JSON telemetry for the mobile app HUD:
```json
{
  "front": 84,
  "left": 130,
  "state": "IDLE",
  "speed": 180,
  "turnSpeed": 155,
  "targetLeft": 0,
  "targetRight": 0,
  "currentLeft": 0,
  "currentRight": 0,
  "clients": 1,
  "uptime": 128,
  "watchdog": 620,
  "autoBrake": true
}
```

### 4. `GET /settings?speed=<int>&turn=<int>&brake=<0|1>&watchdog=<ms>`
Configures operational limits dynamically from the mobile app settings screen.

### 5. `GET /ping`
Lightweight diagnostic endpoint returning `{"pong":true}` to measure network latency (RTT).

---

## 🚀 Flashing to ESP32 via Arduino IDE

1. Open Arduino IDE (v2.x recommended).
2. Go to **File -> Open** and select `firmware/esp32_rover/esp32_rover.ino`.
3. In **Tools -> Board**, select **ESP32 Dev Module** (or your specific ESP32 board).
4. In **Tools -> Port**, select the COM port of your connected ESP32.
5. In **Tools -> Flash Frequency**, select **80MHz**.
6. Click **Upload** (Ctrl + U).
7. Open **Serial Monitor** at **115200 baud** to view initial AP startup logs and IP confirmation.
