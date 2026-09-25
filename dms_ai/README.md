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
      http://192.168.4.1/cmd?val=STOP         "STOP\n"
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

### Dashboard Access
Once running:
- **Local Web HUD**: `http://127.0.0.1:8000`
- **Dual Camera Video Stream**: `http://127.0.0.1:8000/video_feed`
- **Telemetry WebSocket**: `ws://127.0.0.1:8000/ws`
- **Telemetry REST API**: `http://127.0.0.1:8000/telemetry`

---

## ⚙️ Configuration Options

Open `drowsiness_detector.py` to customize:
- `ESP32_HOST_URL = "http://192.168.4.1"`: Wi-Fi gateway IP of the rover.
- `DRIVER_CAM_INDEX = 0`: Camera index for driver webcam.
- `ROAD_SOURCE = 1`: Camera index or video stream for road hazard detection.
- `EYE_CLOSED_THRESHOLD = 0.42`: Sensitivity for detecting closed eyes.
- `CRITICAL_CLOSURE_TIME = 2.0`: Seconds of eye closure before vehicle emergency stop triggers.
- `WARN_CLOSURE_TIME = 1.0`: Seconds of eye closure before audio warning sounds.
