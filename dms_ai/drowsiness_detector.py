import os
os.environ['GLOG_minloglevel'] = '2'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

import asyncio
from contextlib import asynccontextmanager
import json
import threading
import time
import urllib.request
import cv2
import mediapipe as mp
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from starlette.requests import Request
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from ultralytics import YOLO

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    import winsound
    HAS_WINSOUND = True
except ImportError:
    HAS_WINSOUND = False

# ======================= CONFIGURATION =======================
ESP32_HOST_URL = "http://192.168.4.1"
SERIAL_PORT = None
SERIAL_BAUD = 115200

DRIVER_CAM_INDEX = 1   # Built-in laptop webcam
ROAD_SOURCE = None      # Disabled to avoid ceiling fan false alarms

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FACE_MODEL_PATH = os.path.join(SCRIPT_DIR, "face_landmarker.task")
FACE_MODEL_URL = "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task"

POTHOLE_MODEL_PATH = os.path.join(SCRIPT_DIR, "pothole_best.pt")
POTHOLE_MODEL_URL = "https://huggingface.co/peterhdd/pothole-detection-yolov8/resolve/main/best.pt"

EYE_CLOSED_THRESHOLD = 0.42
YAWN_THRESHOLD = 0.50
WARN_CLOSURE_TIME = 1.0
CRITICAL_CLOSURE_TIME = 2.0
YAWN_TRIGGER_TIME = 1.6

templates = Jinja2Templates(directory=os.path.join(SCRIPT_DIR, "templates"))

system_running = True
alarm_level = 0
emergency_halt_active = False
restore_cooldown_until = 0.0
last_emergency_stop_time = 0.0

lock = threading.RLock()
latest_frame = None
pothole_model = None
serial_conn = None

telemetry = {
    "face_detected": False,
    "eye_status": "OPEN",
    "yawning": "NO",
    "drowsiness_level": 5,
    "driver_condition": "SAFE",
    "road_condition": "CLEAR",
    "pothole_detected": False,
    "pothole_count": 0,
    "total_potholes_logged": 0,
    "alarm_status": "OFF",
    "seat_belt": "FASTENED",
    "yawns_detected": 0,
    "eye_blinks": 0,
    "alarms_triggered": 0,
    "fps": 30.0,
    "timestamp": ""
}

system_logs = [
    {"time": time.strftime("%I:%M:%S %p"), "type": "success", "msg": "DMS AI Core Initialized"},
    {"time": time.strftime("%I:%M:%S %p"), "type": "success", "msg": "Wi-Fi & Serial Interlock Ready"}
]

def add_log(msg: str, log_type: str = "success"):
    with lock:
        system_logs.insert(0, {"time": time.strftime("%I:%M:%S %p"), "type": log_type, "msg": msg})
        if len(system_logs) > 30:
            system_logs.pop()

def init_serial():
    global serial_conn
    if not HAS_SERIAL:
        return
    try:
        ports = list(serial.tools.list_ports.comports())
        target_port = SERIAL_PORT
        if target_port is None and len(ports) > 0:
            for p in ports:
                if any(x in p.description.lower() for x in ["cp210", "ch340", "uart", "serial", "esp32"]):
                    target_port = p.device
                    break
            if target_port is None:
                target_port = ports[0].device
        if target_port:
            serial_conn = serial.Serial(target_port, SERIAL_BAUD, timeout=0.1)
            add_log(f"USB Serial Connected ({target_port})", "success")
    except Exception as e:
        print(f"[SERIAL] Wi-Fi HTTP mode active ({e})")

def send_esp32_command(cmd: str):
    def _worker():
        endpoints = [
            f"{ESP32_HOST_URL}/disconnect",
            f"{ESP32_HOST_URL}/cmd?val={cmd}"
        ]
        for url in endpoints:
            try:
                req = urllib.request.Request(url, headers={'User-Agent': 'DMS-AI-Client'})
                with urllib.request.urlopen(req, timeout=0.35) as resp:
                    print(f"[INTERLOCK] Cutoff executed: {url} -> {resp.status}")
                    break
            except Exception:
                pass

        global serial_conn
        if serial_conn and serial_conn.is_open:
            try:
                serial_conn.write(b"STOP\n")
                serial_conn.flush()
            except Exception:
                pass
    threading.Thread(target=_worker, daemon=True).start()

def send_esp32_restore():
    def _worker():
        endpoints = [
            f"{ESP32_HOST_URL}/arm",
            f"{ESP32_HOST_URL}/rearm",
            f"{ESP32_HOST_URL}/restore",
            f"{ESP32_HOST_URL}/cmd?val=ARM"
        ]
        for url in endpoints:
            try:
                req = urllib.request.Request(url, headers={'User-Agent': 'DMS-AI-Client'})
                with urllib.request.urlopen(req, timeout=0.35) as resp:
                    print(f"[INTERLOCK] Power Restored: {url} -> {resp.status}")
                    break
            except Exception:
                pass

        global serial_conn
        if serial_conn and serial_conn.is_open:
            try:
                serial_conn.write(b"ARM\n")
                serial_conn.flush()
            except Exception:
                pass
    threading.Thread(target=_worker, daemon=True).start()

def audio_alert_worker():
    global alarm_level, system_running
    while system_running:
        level = alarm_level
        try:
            if level == 2 and HAS_WINSOUND:
                winsound.Beep(2400, 80)
                time.sleep(0.05)
            elif level == 1 and HAS_WINSOUND:
                winsound.Beep(1400, 100)
                time.sleep(0.1)
            else:
                time.sleep(0.03)
        except Exception:
            time.sleep(0.05)

def download_with_headers(url: str, target_path: str):
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(req) as response, open(target_path, 'wb') as out_file:
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            out_file.write(chunk)

def ensure_models_exist():
    if not os.path.exists(FACE_MODEL_PATH):
        download_with_headers(FACE_MODEL_URL, FACE_MODEL_PATH)

def initialize_driver_camera():
    for idx in [DRIVER_CAM_INDEX, 0, 2]:
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            ret, frame = cap.read()
            if ret and frame is not None:
                return cap
            cap.release()
    return None

def cv_pipeline():
    global system_running, alarm_level, latest_frame, telemetry
    global emergency_halt_active, last_emergency_stop_time, restore_cooldown_until

    ensure_models_exist()
    cap_driver = initialize_driver_camera()

    base_options = python.BaseOptions(model_asset_path=FACE_MODEL_PATH)
    options = vision.FaceLandmarkerOptions(
        base_options=base_options,
        output_face_blendshapes=True,
        num_faces=1
    )
    detector = vision.FaceLandmarker.create_from_options(options)

    eye_closed_start_time = None
    yawn_start_time = None
    is_blinking = False
    is_yawning = False
    blink_timestamps = []
    fps_start_time = time.time()
    frame_counter = 0
    fps = 0.0

    while system_running:
        if cap_driver is not None and cap_driver.isOpened():
            ret_driver, frame_driver = cap_driver.read()
            if not ret_driver or frame_driver is None:
                time.sleep(0.005)
                continue
        else:
            frame_driver = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame_driver, "CAMERA NOT FOUND", (180, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            time.sleep(0.033)

        frame_counter += 1
        curr_time = time.time()
        if curr_time - fps_start_time >= 1.0:
            fps = frame_counter / (curr_time - fps_start_time)
            frame_counter = 0
            fps_start_time = curr_time

        frame_driver = cv2.flip(frame_driver, 1)
        h, w, _ = frame_driver.shape
        rgb_frame = cv2.cvtColor(frame_driver, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        detection_result = detector.detect(mp_image)

        face_detected = False
        eye_is_closed = False
        jaw_is_open = False
        raw_eye_score = 0.0
        raw_mouth_score = 0.0
        eye_closure_duration = 0.0

        if detection_result.face_blendshapes and detection_result.face_landmarks:
            face_detected = True
            blendshapes = {b.category_name: b.score for b in detection_result.face_blendshapes[0]}

            raw_eye_score = max(blendshapes.get("eyeBlinkLeft", 0.0), blendshapes.get("eyeBlinkRight", 0.0))
            raw_mouth_score = blendshapes.get("jawOpen", 0.0)

            eye_is_closed = raw_eye_score > EYE_CLOSED_THRESHOLD
            jaw_is_open = raw_mouth_score > YAWN_THRESHOLD

            landmarks = detection_result.face_landmarks[0]
            xs = [p.x for p in landmarks]
            ys = [p.y for p in landmarks]
            x_min, x_max = max(0, int(min(xs) * w) - 15), min(w, int(max(xs) * w) + 15)
            y_min, y_max = max(0, int(min(ys) * h) - 25), min(h, int(max(ys) * h) + 20)

            color = (0, 0, 255) if eye_is_closed else (0, 255, 0)
            cv2.rectangle(frame_driver, (x_min, y_min), (x_max, y_max), color, 2)

            for idx in [33, 133, 362, 263, 13, 14]:
                pt = landmarks[idx]
                cv2.circle(frame_driver, (int(pt.x * w), int(pt.y * h)), 2, (0, 255, 180), -1)

            if eye_is_closed:
                if not is_blinking:
                    is_blinking = True
                    blink_timestamps.append(curr_time)
                if eye_closed_start_time is None:
                    eye_closed_start_time = curr_time
                eye_closure_duration = curr_time - eye_closed_start_time
            else:
                is_blinking = False
                eye_closed_start_time = None
                eye_closure_duration = 0.0

            if jaw_is_open:
                if yawn_start_time is None:
                    yawn_start_time = curr_time
                if (curr_time - yawn_start_time) >= YAWN_TRIGGER_TIME and not is_yawning:
                    is_yawning = True
                    with lock:
                        telemetry["yawns_detected"] += 1
                    add_log("Yawn Detected", "warning")
            else:
                yawn_start_time = None
                is_yawning = False
        else:
            eye_closed_start_time = None
            yawn_start_time = None

        blink_timestamps = [t for t in blink_timestamps if curr_time - t <= 60]

        status_text = "SAFE"
        alarm_lvl = 0
        drowsiness_pct = int(np.clip((raw_eye_score * 0.7 + raw_mouth_score * 0.3) * 100, 5, 99))
        is_microsleep = (eye_closure_duration >= CRITICAL_CLOSURE_TIME)

        # Trigger emergency stop when eyes remain closed
        if curr_time > restore_cooldown_until:
            if face_detected and is_microsleep:
                status_text = "CRITICAL (MICROSLEEP)"
                alarm_lvl = 2
                drowsiness_pct = max(drowsiness_pct, 95)

                if not emergency_halt_active or (curr_time - last_emergency_stop_time > 2.0):
                    emergency_halt_active = True
                    last_emergency_stop_time = curr_time
                    send_esp32_command("STOP")
                    with lock:
                        telemetry["alarms_triggered"] += 1
                    add_log("CRITICAL: Microsleep detected! Vehicle STOPPED.", "danger")
            elif eye_closure_duration >= WARN_CLOSURE_TIME:
                status_text = "WARNING"
                alarm_lvl = 1
        else:
            status_text = "ARMED / RECOVERING"
            alarm_lvl = 0

        alarm_level = alarm_lvl

        with lock:
            telemetry.update({
                "face_detected": face_detected,
                "eye_status": "CLOSED" if eye_is_closed else "OPEN",
                "yawning": "YES" if jaw_is_open else "NO",
                "drowsiness_level": drowsiness_pct,
                "driver_condition": status_text,
                "road_condition": "CLEAR",
                "pothole_detected": False,
                "pothole_count": 0,
                "alarm_status": "ON" if alarm_level > 0 else "OFF",
                "eye_blinks": len(blink_timestamps),
                "fps": round(fps, 1),
                "timestamp": time.strftime("%I:%M:%S %p")
            })

        cv2.putText(frame_driver, "DRIVER MONITOR", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, "ROAD HAZARD (OFFLINE)", (160, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (100, 100, 100), 2)
        display_frame = np.hstack((frame_driver, placeholder))

        ret, buffer = cv2.imencode(".jpg", display_frame, [cv2.IMWRITE_JPEG_QUALITY, 55])
        if ret:
            with lock:
                latest_frame = buffer.tobytes()

    if cap_driver is not None:
        cap_driver.release()

@asynccontextmanager
async def lifespan(app: FastAPI):
    global system_running
    system_running = True
    init_serial()
    threading.Thread(target=audio_alert_worker, daemon=True).start()
    threading.Thread(target=cv_pipeline, daemon=True).start()
    yield
    system_running = False
    if serial_conn and serial_conn.is_open:
        serial_conn.close()

app = FastAPI(title="AI Driver Assistant DMS", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"])

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse(request=request, name="index.html")

@app.get("/telemetry")
async def get_telemetry():
    with lock:
        return JSONResponse(content={"telemetry": telemetry, "logs": system_logs[:10]})

@app.get("/arm")
@app.post("/arm")
@app.get("/restore")
@app.post("/restore")
async def api_restore():
    global emergency_halt_active, alarm_level, restore_cooldown_until, last_emergency_stop_time
    with lock:
        emergency_halt_active = False
        alarm_level = 0
        restore_cooldown_until = time.time() + 3.0
        last_emergency_stop_time = 0.0
        telemetry["alarm_status"] = "OFF"
        telemetry["driver_condition"] = "SAFE"
    send_esp32_restore()
    add_log("Power Restored & Motors Re-Armed", "success")
    return JSONResponse(content={"status": "ARMED"})

@app.get("/halt")
@app.post("/halt")
@app.get("/stop")
@app.post("/stop")
async def api_halt():
    global emergency_halt_active, alarm_level
    with lock:
        emergency_halt_active = True
        alarm_level = 2
        telemetry["alarm_status"] = "ON"
        telemetry["driver_condition"] = "EMERGENCY HALT"
    send_esp32_command("STOP")
    add_log("Manual Emergency Halt Activated", "danger")
    return JSONResponse(content={"status": "HALTED"})

async def generate_mjpeg():
    global latest_frame, system_running
    try:
        while system_running:
            with lock:
                frame_data = latest_frame
            if frame_data is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame_data + b"\r\n")
            await asyncio.sleep(0.033)
    except (GeneratorExit, asyncio.CancelledError):
        pass

@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(generate_mjpeg(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()

    async def receive_handler():
        try:
            while system_running:
                data = await websocket.receive_text()
                try:
                    msg = json.loads(data)
                    action = msg.get("action", "").lower()
                except Exception:
                    action = data.strip().lower()
                if action in ["arm", "restore"]:
                    await api_restore()
                elif action in ["halt", "stop", "disconnect"]:
                    await api_halt()
        except Exception:
            pass

    rx_task = asyncio.create_task(receive_handler())
    try:
        while system_running:
            with lock:
                payload = {"telemetry": telemetry, "logs": system_logs[:8]}
            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(0.06)
    except (WebSocketDisconnect, asyncio.CancelledError):
        pass
    finally:
        rx_task.cancel()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")