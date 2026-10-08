# AI Driver Monitoring System (DMS) & Road Hazard Detector

This module runs on your laptop or companion computer with a camera to monitor the driver for **drowsiness, eye closure, fatigue, and yawns**, while simultaneously using **YOLOv8** to scan the road ahead for **potholes**.

When microsleep or severe fatigue is detected, it automatically transmits emergency stop commands to the ESP32 vehicle over **both Wi-Fi HTTP and USB Serial**.

---

## ⚡ Integration with ESP32 Rover & Expo App

```text
  [ Laptop Webcam ] ──> MediaPipe Face Landmarker (Blink / Yawn / Eye Closure)
                                │
  [ Road Camera ]   ──> YOLOv8 Pothole Detector (Road Hazards)
                                │
                 [ Fatigue >= 80% OR Eye Closed >= 2.0s ]
                                │
                 ┌──────────────┴──────────────┐
                 ▼                             ▼
       [ Wi-Fi HTTP Dispatch ]        [ USB Serial Dispatch ]
 http://esp32-rover.local/cmd?val=STOP         "STOP\n"
                 │                             │
                 └──────────────┬──────────────┘
                                ▼
                   [ ESP32 BTS7960 Driver ]
                 - Floats all MOSFET gates (R_EN/L_EN = LOW)
                 - Sets state to STATE_HALTED
                 - Requires user tap on "ARM" to restore
                                ▲
                                │
                   [ Expo Mobile Cockpit ]
                 - Receives live telemetry & video feed
                 - Triggers flashing red alert HUD
                 - Provides "RESTORE POWER" button
```

---

## 🚀 How to Run the DMS AI System

### Quick Start (Windows)
Double-click `run_dms.bat` or run in terminal:
```bash
cd dms_ai
pip install -r requirements.txt
python drowsiness_detector.py
```

### Dashboard Access Across Common Network
Once launched, the terminal displays your Laptop's LAN IP:
- **Local Laptop Browser**: `http://localhost:8000`
- **Mobile Phone / LAN Browser**: `http://<laptop-ip>:8000` (e.g. `http://192.168.1.50:8000`)
- **Dual Camera Video Stream**: `http://<laptop-ip>:8000/video_feed`
- **Telemetry WebSocket**: `ws://<laptop-ip>:8000/ws`
- **Telemetry REST API**: `http://<laptop-ip>:8000/telemetry`

---

## ⚙️ Configuration & Auto-Discovery

The DMS AI connects to the ESP32 rover on your common network:
- **Target URL Default**: `http://esp32-rover.local` (or whatever is in `config.json`).
- **LAN Auto-Discovery**: Automatically listens for or broadcasts UDP discovery packets on port 4210.
- **Web UI URL Config**: Click the **ROVER** badge in the top bar of the DMS web dashboard to view or change the target Rover IP at any time without editing code.
