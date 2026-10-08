# 🚗 Driver Assistance ESP32 Rover

A prototype **Driver Assistance and Vehicle Safety System** built using ESP32, BTS7960 motor drivers, an MQ-3 alcohol sensor, AI-based Driver Monitoring, pothole detection, and a React Native Expo mobile cockpit.

The main idea of this project is to monitor both the **driver** and the **road environment**, and provide an appropriate safety response when an unsafe condition is detected.

The system currently provides:

* 🍺 Alcohol detection and vehicle lock
* 😴 Driver drowsiness and microsleep detection
* 🕳️ AI-based pothole detection
* 🚨 Emergency safety intervention
* 🎮 Manual rover control through a mobile application
* 📡 Wi-Fi communication between the ESP32, laptop and mobile application
* 💡 Visual and audio safety indications

This is a **prototype for a Driver Assistance System**, demonstrating how driver monitoring, road monitoring and vehicle control can work together.

---

# 📌 How the System Works

The system has three main parts:

```text
        DRIVER
           │
           ▼
    ┌──────────────┐
    │  DMS Camera  │
    └──────┬───────┘
           │
           ▼
    ┌──────────────┐
    │   DMS AI     │
    │ Python + AI  │
    └──────┬───────┘
           │
          Wi-Fi
           │
           ▼
┌─────────────────────┐
│        ESP32        │
│ Main Vehicle        │
│ Safety Controller   │
└─────────┬───────────┘
          │
          ▼
    ┌─────────────┐
    │  BTS7960    │
    │ Motor       │
    │ Drivers     │
    └──────┬──────┘
           │
           ▼
         Motors


┌─────────────────────┐
│   Mobile Cockpit    │
│ React Native + Expo │
└──────────┬──────────┘
           │
          Wi-Fi
           │
           ▼
         ESP32
```

The **ESP32 is the main embedded controller**.

The mobile application is used for manual control and system monitoring.

The laptop runs the AI Driver Monitoring System.

---

# 🎯 Main Safety Functions

## 1. 🍺 Alcohol Detection

An **MQ-3 alcohol gas sensor** is connected to the ESP32.

The ESP32 reads the sensor's analog output and compares it with the configured prototype threshold.

The current prototype threshold is:

```text
ADC Threshold = 3000
```

The basic process is:

```text
MQ-3 Sensor
     ↓
ESP32 ADC Reading
     ↓
Compare with 3000
     ↓
Unsafe condition detected
     ↓
ALCOHOL_LOCKED
     ↓
Motor movement disabled
```

When the alcohol condition is detected, the ESP32 prevents the vehicle from being driven.

The system can also activate the configured visual/audio safety indication.

> The value `3000` is a prototype calibration threshold for this particular sensor and setup. It is not a BAC value.

---

# 2. 😴 Driver Drowsiness & Microsleep Detection

The Driver Monitoring System runs on a laptop using a camera.

The system uses computer vision to monitor the driver's face.

The DMS uses:

* Python
* OpenCV
* MediaPipe
* FastAPI

The system analyses facial features such as:

* eye closure
* eye closure duration
* mouth opening
* yawning

A short eye closure is treated differently from prolonged eye closure.

When critical microsleep is detected:

```text
Driver Camera
      ↓
OpenCV
      ↓
MediaPipe Facial Landmarks
      ↓
Eye / Mouth Analysis
      ↓
Critical Microsleep
      ↓
Safety Command
      ↓
Wi-Fi
      ↓
ESP32
      ↓
Vehicle Safety Intervention
      ↓
Vehicle Stopped
```

The purpose is to prevent the vehicle from continuing normal movement when the driver is detected to be in a critical drowsy state.

---

# 3. 🕳️ Pothole Detection

A road-facing camera provides images to the AI system.

The pothole detection model analyses the camera frames and identifies potholes.

```text
Road Camera
     ↓
AI Vision Model
     ↓
Pothole Detection
     ↓
Warning / Display
```

The detected pothole information is displayed through the Driver Monitoring interface/mobile cockpit.

The current prototype uses pothole detection as a **driver-assistance warning feature**.

---

# 4. 🎮 Manual Vehicle Control

The rover can be manually controlled using the React Native Expo mobile application.

The cockpit provides controls such as:

* Forward
* Reverse
* Left
* Right
* Diagonal movement
* Virtual joystick
* Speed control
* Emergency stop

The communication path is:

```text
Mobile Application
        ↓
       Wi-Fi
        ↓
       ESP32
        ↓
   BTS7960 Drivers
        ↓
      Motors
```

The mobile application does **not directly control the motors electrically**.

It sends digital commands to the ESP32.

The ESP32 receives the command, checks the current safety state, and then controls the motor drivers.

---

# 📡 Wi-Fi Communication

Wi-Fi is used as the communication medium between the different parts of the system.

The project can operate using a common Wi-Fi network.

```text
        Wi-Fi Network
       /      |       \
      /       |        \
 ESP32      Laptop     Phone
              │
             DMS
```

The mobile application communicates with the ESP32 using HTTP requests.

The DMS running on the laptop can also communicate with the ESP32 through the network.

This allows a safety event detected by the DMS to reach the vehicle controller.

---

# 🧠 Safety Decision Architecture

The system follows:

```text
SENSING
   ↓
PROCESSING
   ↓
SAFETY DECISION
   ↓
VEHICLE RESPONSE
```

Examples:

### Alcohol

```text
MQ-3
 ↓
ESP32
 ↓
Alcohol condition
 ↓
Vehicle locked
```

### Microsleep

```text
Camera
 ↓
DMS AI
 ↓
Critical microsleep
 ↓
ESP32
 ↓
Safety intervention
 ↓
Vehicle stopped
```

### Pothole

```text
Road Camera
 ↓
AI Model
 ↓
Pothole detected
 ↓
Driver warning
```

---

# 📁 Project Structure

The project structure remains unchanged:

```text
Driver Assistance ESP32 Rover/
│
├── firmware/
│   ├── esp32_rover/
│   │   └── esp32_rover.ino
│   │
│   └── README.md
│
├── mobile_app/
│   ├── src/
│   │   ├── components/
│   │   │   ├── Header.tsx
│   │   │   ├── StatusBanner.tsx
│   │   │   ├── DmsAiCard.tsx
│   │   │   ├── DPadController.tsx
│   │   │   ├── VirtualJoystick.tsx
│   │   │   ├── SpeedControl.tsx
│   │   │   ├── EmergencyButton.tsx
│   │   │   └── SettingsModal.tsx
│   │   │
│   │   ├── services/
│   │   │   └── RoverService.ts
│   │   │
│   │   └── types/
│   │       └── index.ts
│   │
│   ├── App.tsx
│   ├── package.json
│   └── README.md
│
├── dms_ai/
│   ├── drowsiness_detector.py
│   ├── templates/
│   │   └── index.html
│   ├── requirements.txt
│   ├── run_dms.bat
│   └── README.md
│
└── README.md
```

---

# 📂 Folder Description

## `firmware/`

Contains the ESP32 firmware.

The ESP32 is responsible for:

* receiving commands
* controlling motors
* reading the MQ-3 sensor
* applying alcohol safety lock
* managing vehicle safety states
* emergency motor cutoff
* safety watchdog
* controlling safety indicators
* communicating through Wi-Fi

Main firmware:

```text
firmware/esp32_rover/esp32_rover.ino
```

---

## `mobile_app/`

Contains the React Native + Expo mobile cockpit.

The application provides:

* manual rover control
* virtual joystick
* D-pad
* speed control
* ESP32 connection status
* DMS status
* pothole information
* emergency stop
* system configuration

The application acts as the project's **Human-Machine Interface (HMI)**.

---

## `dms_ai/`

Contains the Python-based Driver Monitoring System.

It handles:

* camera input
* facial landmark detection
* drowsiness detection
* microsleep detection
* yawning detection
* pothole detection
* DMS web dashboard
* communication with the ESP32

Main file:

```text
dms_ai/drowsiness_detector.py
```

---

# 🔌 Hardware Pinout

| Component      | Function      | ESP32 GPIO |
| -------------- | ------------- | ---------: |
| Left BTS7960   | RPWM          |    GPIO 19 |
| Left BTS7960   | LPWM          |    GPIO 18 |
| Left BTS7960   | R_EN          |    GPIO 21 |
| Left BTS7960   | L_EN          |    GPIO 22 |
| Right BTS7960  | RPWM          |    GPIO 25 |
| Right BTS7960  | LPWM          |    GPIO 23 |
| Right BTS7960  | R_EN          |    GPIO 27 |
| Right BTS7960  | L_EN          |    GPIO 32 |
| MQ-3           | Analog Output |    GPIO 34 |
| Main LED Strip | Data          |     GPIO 5 |
| Neon LED Strip | Data          |    GPIO 14 |
| Buzzer         | Output        |     GPIO 4 |

---

# ⚙️ Motor Control

The ESP32 controls the motors through two BTS7960 high-current H-bridge motor drivers.

```text
ESP32
 │
 ├── Left BTS7960 ── Left Motor
 │
 └── Right BTS7960 ─ Right Motor
```

The system uses differential drive, allowing the left and right motors to be controlled independently.

Motor control uses PWM.

```text
PWM Frequency: 5 kHz
PWM Resolution: 8-bit
PWM Range: 0–255
```

The PWM value controls the effective motor drive level.

The prototype does not directly measure motor RPM because motor encoders are not used.

---

# 🛡️ Safety Features

## Emergency Motor Cutoff

When an emergency safety condition occurs, the ESP32 disables the motor-driver enable lines.

This prevents normal motor drive from continuing.

---

## Communication Watchdog

The ESP32 uses a safety watchdog.

If the control application stops communicating for approximately:

```text
650 ms
```

the ESP32 automatically stops motor drive.

This protects the vehicle if communication is unexpectedly interrupted.

---

## Alcohol Safety Lock

When the MQ-3 reading crosses the configured threshold:

```text
3000
```

the ESP32 enters the alcohol safety lock state.

Motor movement is disabled until the safety condition is cleared.

---

## Safety Indicators

The system uses visual/audio indicators to communicate safety conditions.

These can include:

* LED indication
* neon LED status
* buzzer alerts

---

# 🔄 Vehicle States

The ESP32 manages different operating states.

```text
IDLE
MANUAL
HALTED
ALCOHOL_LOCKED
```

### IDLE

The vehicle is powered but is not actively moving.

### MANUAL

The vehicle is being controlled through the mobile cockpit.

### HALTED

Vehicle movement has been stopped because of an emergency or safety condition.

### ALCOHOL_LOCKED

The alcohol detection system has detected an unsafe condition and vehicle movement is disabled.

---

# 🚀 How to Clone the Project

Make sure Git is installed.

Check:

```bash
git --version
```

Clone the repository:

```bash
git clone https://github.com/VarshaJ004/esp32-safety-rover.git
```

Enter the project:

```bash
cd esp32-safety-rover
```

You should see:

```text
firmware/
mobile_app/
dms_ai/
README.md
```

---

# 🛠️ Requirements

## Hardware

* ESP32 development board
* 2 × BTS7960 motor drivers
* DC motors
* MQ-3 alcohol sensor
* LED strips
* Buzzer
* Rover chassis
* Battery/power supply
* Laptop with camera
* Smartphone

## Software

* Arduino IDE
* ESP32 Arduino board package
* Python 3.x
* Node.js
* npm
* Expo Go
* Git

---

# 1️⃣ Run the ESP32 Firmware

Open:

```text
firmware/esp32_rover/esp32_rover.ino
```

using Arduino IDE.

Configure your Wi-Fi:

```cpp
const char *ssid = "Your-WiFi-Name";
const char *password = "Your-WiFi-Password";
```

Connect the ESP32 using USB.

Select the correct ESP32 board and COM port.

Upload the firmware.

Open Serial Monitor:

```text
115200 baud
```

The ESP32 will display its network information.

---

# 2️⃣ Run the DMS AI

Open a terminal inside:

```bash
cd dms_ai
```

Create a virtual environment:

```bash
python -m venv venv
```

Activate it on Windows:

```bash
venv\Scripts\activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

Run:

```bash
python drowsiness_detector.py
```

On Windows, you can also use:

```text
run_dms.bat
```

The DMS dashboard runs at:

```text
http://localhost:8000
```

---

# 3️⃣ Run the Mobile Application

Open another terminal:

```bash
cd mobile_app
```

Install dependencies:

```bash
npm install
```

Start Expo:

```bash
npm start
```

or:

```bash
npx expo start
```

Open **Expo Go** on the phone and scan the QR code.

Make sure the phone and laptop are connected to the same Wi-Fi network.

---

# 📱 Configure the Mobile App

Open:

```text
System Config
```

Set the ESP32 address as the **Rover Gateway**.

For example:

```text
http://192.168.x.x
```

Set the DMS laptop address as the **DMS Host**:

```text
http://<laptop-ip>:8000
```

Then use the connection test options to verify communication.

---

# ▶️ Complete Run Order

For a complete demonstration:

### 1. Start ESP32

```text
Power ESP32
↓
Connect to Wi-Fi
↓
Confirm IP address
```

### 2. Start DMS

```text
Run drowsiness_detector.py
↓
Open DMS dashboard
```

### 3. Start Mobile App

```text
npm start
↓
Open Expo Go
↓
Configure ESP32 + DMS addresses
```

### 4. Test the System

```text
Manual Control
      ↓
Alcohol Detection
      ↓
Drowsiness Detection
      ↓
Microsleep Safety Response
      ↓
Pothole Detection
```

---

# 🧪 Recommended Testing

Test each feature separately before running the complete demonstration.

### Manual Control

```text
Mobile App → Wi-Fi → ESP32 → BTS7960 → Motors
```

### Alcohol Detection

```text
MQ-3 → ESP32 → Alcohol Lock → Motor Disabled
```

### Drowsiness Detection

```text
Camera → Python DMS → Drowsiness Analysis
```

### Microsleep Safety

```text
Camera → DMS → Safety Command → ESP32 → Vehicle Stopped
```

### Pothole Detection

```text
Road Camera → AI Model → Pothole Detection → Warning
```

---

# 🧩 Technologies Used

### Embedded

* ESP32
* Arduino
* BTS7960
* PWM
* MQ-3

### AI / Computer Vision

* Python
* OpenCV
* MediaPipe
* YOLO-based pothole detection
* FastAPI

### Mobile

* React Native
* Expo
* TypeScript

### Communication

* Wi-Fi
* HTTP / REST API
* mDNS

---

# 🎯 Project Objective

The objective of the **Driver Assistance ESP32 Rover** is to demonstrate how a vehicle can use information about the **driver and the road environment** to improve safety.

The system combines:

```text
Driver Monitoring
       +
Road Monitoring
       +
Vehicle Control
       ↓
Safety Response
```

The project demonstrates three different safety responses:

```text
Alcohol
   ↓
Prevent unsafe vehicle operation

Microsleep
   ↓
Emergency safety intervention

Pothole
   ↓
Driver warning
```

The ESP32 acts as the central vehicle controller, the DMS performs AI-based monitoring, and the mobile application provides the Human-Machine Interface.

---

# 🔮 Future Scope

The prototype can be extended in the future with:

* automotive-grade alcohol sensing
* advanced Driver Monitoring Systems
* improved pothole classification
* additional environmental perception
* lane detection
* radar/LiDAR integration
* automotive ECUs
* CAN communication
* production-grade braking and steering systems

---

# 👥 System Architecture

```text
                    DRIVER
                       │
                       ▼
                ┌─────────────┐
                │    Camera   │
                └──────┬──────┘
                       │
                       ▼
                ┌─────────────┐
                │   DMS AI    │
                │ Python + AI │
                └──────┬──────┘
                       │
                      Wi-Fi
                       │
                       ▼
┌────────────────┐   ┌───────────────┐
│ Mobile Cockpit │◄─►│     ESP32     │
│ React Native   │   │ Main Controller│
└────────────────┘   └───────┬───────┘
                             │
                      ┌──────┴──────┐
                      │             │
                      ▼             ▼
                 BTS7960        MQ-3
                 Drivers        Sensor
                      │
                      ▼
                    Motors
```

**Driver Assistance ESP32 Rover** is a prototype demonstrating the integration of driver monitoring, road hazard detection, wireless communication and embedded vehicle control in a single safety-oriented system.
