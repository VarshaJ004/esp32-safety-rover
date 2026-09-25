# 🏎️ ESP32 Autonomous & Manual Safety Rover Controller

An end-to-end, high-performance robotics control system for ESP32 with dual BTS7960 43A motor drivers, ultrasonic obstacle avoidance, microsleep emergency safety cutoff, and a dedicated **React Native Expo Mobile Cockpit**.

---

## 📁 Project Structure

```
esp32 rover/
├── firmware/
│   ├── esp32_rover/
│   │   └── esp32_rover.ino        # Upgraded ESP32 Arduino firmware (CORS, REST API, Differential Drive)
│   └── README.md                  # Complete wiring schematics, pinouts & hardware specs
│
├── mobile_app/
│   ├── src/
│   │   ├── components/
│   │   │   ├── Header.tsx          # Real-time connection badge & ping telemetry
│   │   │   ├── StatusBanner.tsx    # State indicator & microsleep re-arm action
│   │   │   ├── DmsAiCard.tsx       # Live AI Driver Alertness, Drowsiness meter & Pothole display
│   │   │   ├── SonarDisplay.tsx    # Dual ultrasonic sonar radar HUD with proximity alerts
│   │   │   ├── DPadController.tsx  # 8-direction tactile D-Pad with diagonal steering
│   │   │   ├── VirtualJoystick.tsx # Proportional 2D joystick with differential drive physics
│   │   │   ├── SpeedControl.tsx    # Eco, Mid, Sport presets & PWM fine adjustment
│   │   │   ├── EmergencyButton.tsx # Prominent tyre disconnect button (floats MOSFETs)
│   │   │   └── SettingsModal.tsx   # Config modal, connection tester & demo mode
│   │   ├── services/
│   │   │   └── RoverService.ts     # Network service with timeout guard, DMS sync & mock simulator
│   │   └── types/
│   │       └── index.ts            # TypeScript interfaces and data models
│   ├── App.tsx                     # Main application cockpit container
│   ├── package.json
│   └── README.md                   # Mobile app setup & usage instructions
│
├── dms_ai/
│   ├── drowsiness_detector.py     # MediaPipe FaceLandmarker + YOLOv8 Pothole detection + FastAPI
│   ├── templates/index.html       # Web dashboard with live camera feed and charts
│   ├── requirements.txt           # Python dependencies
│   ├── run_dms.bat                # 1-click Windows launcher
│   └── README.md                  # DMS setup & integration instructions
│
└── README.md                       # Master project overview
```

---

## ⚡ Hardware Pinout Summary

| Component | Function | ESP32 GPIO | Description |
| :--- | :--- | :--- | :--- |
| **Left BTS7960** | RPWM | `GPIO 19` | Left Forward PWM (5 kHz) |
| | LPWM | `GPIO 18` | Left Reverse PWM (5 kHz) |
| | R_EN | `GPIO 21` | Left Forward Enable (MOSFET Gate) |
| | L_EN | `GPIO 22` | Left Reverse Enable (MOSFET Gate) |
| **Right BTS7960** | RPWM | `GPIO 25` | Right Forward PWM (5 kHz) |
| | LPWM | `GPIO 23` | Right Reverse PWM (5 kHz) |
| | R_EN | `GPIO 27` | Right Forward Enable (MOSFET Gate) |
| | L_EN | `GPIO 32` | Right Reverse Enable (MOSFET Gate) |
| **Front Ultrasonic**| Trig | `GPIO 4` | Front Sonar Trigger |
| | Echo | `GPIO 16` | Front Sonar Echo (via 1k/2k voltage divider) |
| **Left Ultrasonic** | Trig | `GPIO 26` | Left Sonar Trigger |
| | Echo | `GPIO 17` | Left Sonar Echo (via 1k/2k voltage divider) |

---

## 🚀 Quick Start Guide

### 1. Flash the ESP32 Firmware
1. Open Arduino IDE and load `firmware/esp32_rover/esp32_rover.ino`.
2. Connect your ESP32 board via USB.
3. Select board `ESP32 Dev Module` and click **Upload**.
4. The ESP32 will broadcast a Wi-Fi Hotspot:
   - **SSID**: `ESP32-Safety-Car`
   - **Password**: `12345678`
   - **IP Address**: `http://192.168.4.1`

### 2. Launch the Mobile App
1. Install **Expo Go** on your iOS or Android device.
2. In your terminal, run:
   ```bash
   cd mobile_app
   npm start
   ```
3. Connect your phone's Wi-Fi to `ESP32-Safety-Car`.
4. Scan the QR code in the terminal to open the dedicated rover cockpit in **Expo Go**.
5. Alternatively, run `npm run web` to preview the controller in your desktop browser!

---

## 🛡️ Safety & Fail-Safe Features

1. **Gate Isolation (Tyre Disconnect)**: When emergency stop or microsleep detection triggers, the ESP32 disables all `R_EN` and `L_EN` lines, completely floating the MOSFET gates and cutting all current to the motors.
2. **Safety Watchdog (650ms)**: If phone goes out of Wi-Fi range or the app is minimized, the ESP32 automatically cuts motor drive after 650ms of silence.
3. **Obstacle Auto-Brake**: If the front ultrasonic sensor detects an obstacle closer than 18cm while driving forward, the car automatically brakes.
4. **Soft Acceleration Ramping**: Limits rapid current draw spikes to protect the BTS7960 drivers and the battery.
