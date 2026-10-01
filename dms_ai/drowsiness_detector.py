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

DRIVER_CAM_INDEX = 1   # Set to 0 or 1 depending on your webcam index
ROAD_SOURCE = None      # Road cam index (set to None if testing with single cam)

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

# System State Machine
system_running = True
alarm_level = 0
emergency_halt_active = False
alcohol_lock_active = False
restore_cooldown_until = 0.0
last_emergency_stop_time = 0.0

lock = threading.RLock()
latest_frame = None
pothole_model = None
serial_conn = None

pothole_ai_lock = threading.Lock()
latest_road_frame_for_ai = None
cached_pothole_boxes = []

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
    "alcohol_level": 0,
    "alcohol_detected": False,
    "fps": 30.0,
    "timestamp": ""
}

system_logs = [
    {"time": time.strftime("%I:%M:%S %p"), "type": "success", "msg": "DMS AI Core Initialized"},
    {"time": time.strftime("%I:%M:%S %p"), "type": "success", "msg": "Zero-Lag Video Pipeline Active"}
]

def add_log(msg: str, log_type: str = "success"):
    with lock:
        system_logs.insert(0, {"time": time.strftime("%I:%M:%S %p"), "type": log_type, "msg": msg})
        if len(system_logs) > 30:
            system_logs.pop()

class ThreadedCamera:
    """Consistently pulls hardware frames in background to clear OpenCV buffer lag."""
    def __init__(self, src, width=640, height=480):
        self.src = src
        self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.frame = frame
                    self.ret = ret
            else:
                time.sleep(0.01)

    def read(self):
        with self.lock:
            if self.frame is not None:
                return self.ret, self.frame.copy()
            return False, None

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=0.3)
        self.cap.release()

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
        print(f"[SERIAL] Running in Wi-Fi HTTP mode: {e}")

def send_esp32_command(cmd: str):
    def _worker():
        url = f"{ESP32_HOST_URL}/cmd?val={cmd}"
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'DMS-AI-Client'})
            with urllib.request.urlopen(req, timeout=2.0) as resp:
                print(f"[HTTP] Command '{cmd}' delivered: {resp.status}")
        except Exception as e:
            pass

        global serial_conn
        if serial_conn and serial_conn.is_open:
            try:
                serial_conn.write(f"{cmd}\n".encode())
                serial_conn.flush()
            except Exception:
                pass
    threading.Thread(target=_worker, daemon=True).start()

def send_esp32_restore():
    send_esp32_command("ARM")

def esp32_telemetry_worker():
    global system_running, alcohol_lock_active, emergency_halt_active, alarm_level, restore_cooldown_until, last_emergency_stop_time, serial_conn
    last_logged_alcohol_state = False

    while system_running:
        # PYTHON SERIAL FIX: Flush the entire buffer so [ARMED] isn't lagging behind MQ3 logs
        if serial_conn and serial_conn.is_open:
            try:
                while serial_conn.in_waiting > 0:
                    line = serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if "[ARMED]" in line:
                        with lock:
                            emergency_halt_active = False
                            alarm_level = 0
                            restore_cooldown_until = time.time() + 4.0
                            last_emergency_stop_time = 0.0
                            telemetry["driver_condition"] = "SAFE"
                            telemetry["alarm_status"] = "OFF"
                            add_log("Synced: Rover Armed via Hardware Serial", "success")
            except Exception:
                pass

        try:
            url = f"{ESP32_HOST_URL}/status"
            req = urllib.request.Request(url, headers={'User-Agent': 'DMS-AI-Client'})
            with urllib.request.urlopen(req, timeout=1.5) as resp:
                if resp.status == 200:
                    data = json.loads(resp.read().decode('utf-8'))
                    alc_val = data.get("alcohol", 0)
                    alc_detected = data.get("alcoholDetected", False)
                    esp_state = data.get("state", "IDLE")

                    with lock:
                        telemetry["alcohol_level"] = alc_val
                        telemetry["alcohol_detected"] = alc_detected
                        alcohol_lock_active = alc_detected

                        if esp_state in ["IDLE", "MANUAL"] and emergency_halt_active:
                            emergency_halt_active = False
                            alarm_level = 0
                            restore_cooldown_until = time.time() + 4.0
                            last_emergency_stop_time = 0.0
                            telemetry["driver_condition"] = "SAFE"
                            telemetry["alarm_status"] = "OFF"
                            add_log("Synced: Rover Armed via Mobile App", "success")

                    if alc_detected and not last_logged_alcohol_state:
                        last_logged_alcohol_state = True
                        add_log(f"ALCOHOL DETECTED ({alc_val}): Controls Locked", "danger")
                    elif not alc_detected and last_logged_alcohol_state:
                        last_logged_alcohol_state = False
                        add_log("Alcohol cleared. Ready to Re-Arm.", "success")
        except Exception:
            pass
        time.sleep(0.15)

def audio_alert_worker():
    global alarm_level, system_running, alcohol_lock_active
    while system_running:
        level = alarm_level
        if alcohol_lock_active:
            level = max(level, 2)
            
        try:
            if level == 2 and HAS_WINSOUND:
                winsound.Beep(2400, 80)
                time.sleep(0.05)
            elif level == 1 and HAS_WINSOUND:
                winsound.Beep(1400, 100)
                time.sleep(0.1)
            else:
                time.sleep(0.05)
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
    if ROAD_SOURCE is not None and not os.path.exists(POTHOLE_MODEL_PATH):
        try:
            download_with_headers(POTHOLE_MODEL_URL, POTHOLE_MODEL_PATH)
        except Exception as e:
            print(f"[MODEL] Could not download pothole weights: {e}")

def pothole_ai_worker():
    global system_running, pothole_model, latest_road_frame_for_ai, cached_pothole_boxes
    while system_running:
        if ROAD_SOURCE is None or pothole_model is None:
            time.sleep(0.5)
            continue

        frame_to_process = None
        with pothole_ai_lock:
            if latest_road_frame_for_ai is not None:
                frame_to_process = latest_road_frame_for_ai.copy()

        if frame_to_process is not None:
            try:
                results = pothole_model(frame_to_process, conf=0.35, imgsz=416, verbose=False)
                new_boxes = []
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    new_boxes.append((x1, y1, x2, y2, conf))
                with pothole_ai_lock:
                    cached_pothole_boxes = new_boxes
            except Exception:
                pass
        time.sleep(0.05)

def cv_pipeline():
    global system_running, alarm_level, latest_frame, telemetry
    global emergency_halt_active, last_emergency_stop_time, restore_cooldown_until
    global pothole_model, latest_road_frame_for_ai, cached_pothole_boxes, alcohol_lock_active

    ensure_models_exist()

    if ROAD_SOURCE is not None and os.path.exists(POTHOLE_MODEL_PATH):
        try:
            pothole_model = YOLO(POTHOLE_MODEL_PATH)
        except Exception as e:
            pass

    threading.Thread(target=pothole_ai_worker, daemon=True).start()

    cap_driver = ThreadedCamera(DRIVER_CAM_INDEX, 640, 480)
    cap_road = ThreadedCamera(ROAD_SOURCE, 640, 480) if ROAD_SOURCE is not None else None

    cached_road_placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(cached_road_placeholder, "ROAD CAM NOT CONNECTED", (160, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (90, 90, 90), 2)

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

    last_pothole_seen_time = 0.0
    pothole_active_logged = False
    last_eyes_reopened_logged = False

    while system_running:
        curr_time = time.time()

        ret_driver, frame_driver = cap_driver.read()
        if not ret_driver or frame_driver is None:
            time.sleep(0.005)
            continue

        if cap_road is not None and cap_road.isOpened():
            ret_road, frame_road = cap_road.read()
            if not ret_road or frame_road is None:
                frame_road = cached_road_placeholder.copy()
            with pothole_ai_lock:
                latest_road_frame_for_ai = frame_road
        else:
            frame_road = cached_road_placeholder

        frame_counter += 1
        if curr_time - fps_start_time >= 1.0:
            fps = frame_counter / (curr_time - fps_start_time)
            frame_counter = 0
            fps_start_time = curr_time

        frame_driver = cv2.flip(frame_driver, 1)
        h, w, _ = frame_driver.shape

        small_driver = cv2.resize(frame_driver, (320, 240))
        rgb_frame = cv2.cvtColor(small_driver, cv2.COLOR_BGR2RGB)
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
                last_eyes_reopened_logged = False
            else:
                if emergency_halt_active and not last_eyes_reopened_logged and eye_closure_duration > 0:
                    add_log("Eyes Reopened: Vehicle remains parked until Re-Armed", "warning")
                    last_eyes_reopened_logged = True
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

        if alcohol_lock_active:
            status_text = "ALCOHOL LOCKED"
            alarm_lvl = 2
        elif curr_time < restore_cooldown_until:
            status_text = "SYSTEM RESTORED / COOLDOWN"
            alarm_lvl = 0
        elif emergency_halt_active:
            if not eye_is_closed:
                status_text = "DRIVER AWAKE - PARKED (CLICK RESTORE)"
                alarm_lvl = 0
            else:
                status_text = "PARKED & LOCKED: EYES CLOSED"
                alarm_lvl = 2
        else:
            if face_detected and is_microsleep:
                status_text = "CRITICAL (MICROSLEEP AUTO-PARK)"
                alarm_lvl = 2
                drowsiness_pct = max(drowsiness_pct, 95)

                if not emergency_halt_active or (curr_time - last_emergency_stop_time > 2.0):
                    emergency_halt_active = True
                    last_emergency_stop_time = curr_time
                    send_esp32_command("STOP")
                    with lock:
                        telemetry["alarms_triggered"] += 1
                    add_log("CRITICAL: Microsleep detected! Emergency halt initiated.", "danger")
            elif eye_closure_duration >= WARN_CLOSURE_TIME:
                status_text = "WARNING"
                alarm_lvl = 1
            else:
                status_text = "SAFE"
                alarm_lvl = 0

        alarm_level = alarm_lvl

        potholes_found = 0
        pothole_detected = False
        if ROAD_SOURCE is not None:
            with pothole_ai_lock:
                boxes_to_draw = list(cached_pothole_boxes)
            potholes_found = len(boxes_to_draw)
            pothole_detected = potholes_found > 0
            
            for (x1, y1, x2, y2, conf) in boxes_to_draw:
                cv2.rectangle(frame_road, (x1, y1), (x2, y2), (0, 69, 255), 2)
                cv2.putText(frame_road, f"Pothole {int(conf*100)}%", (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 69, 255), 2)

            if pothole_detected:
                if not pothole_active_logged and (curr_time - last_pothole_seen_time > 2.5):
                    with lock:
                        telemetry["total_potholes_logged"] += potholes_found
                    add_log(f"Pothole Detected ({potholes_found} in path)", "warning")
                    pothole_active_logged = True
                    last_pothole_seen_time = curr_time
            else:
                pothole_active_logged = False

        with lock:
            telemetry.update({
                "face_detected": face_detected,
                "eye_status": "CLOSED" if eye_is_closed else "OPEN",
                "yawning": "YES" if jaw_is_open else "NO",
                "drowsiness_level": drowsiness_pct,
                "driver_condition": status_text,
                "road_condition": "POTHOLE DETECTED" if pothole_detected else "CLEAR",
                "pothole_detected": pothole_detected,
                "pothole_count": potholes_found,
                "alarm_status": "ON" if (alarm_level > 0 or alcohol_lock_active) else "OFF",
                "eye_blinks": len(blink_timestamps),
                "fps": round(fps, 1),
                "timestamp": time.strftime("%I:%M:%S %p")
            })

        cv2.putText(frame_driver, f"DRIVER MONITOR - FPS: {round(fps, 1)}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_road, "ROAD MONITOR", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 240, 255), 2)

        display_frame = np.hstack((frame_driver, frame_road))
        ret, buffer = cv2.imencode(".jpg", display_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ret:
            with lock:
                latest_frame = buffer.tobytes()

    cap_driver.release()
    if cap_road is not None:
        cap_road.release()

@asynccontextmanager
async def lifespan(app: FastAPI):
    global system_running
    system_running = True
    init_serial()
    threading.Thread(target=audio_alert_worker, daemon=True).start()
    threading.Thread(target=esp32_telemetry_worker, daemon=True).start()
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
        return JSONResponse(content={"telemetry": telemetry, "logs": system_logs[:12]})

@app.get("/arm")
@app.post("/arm")
@app.get("/restore")
@app.post("/restore")
async def api_restore():
    global emergency_halt_active, alarm_level, restore_cooldown_until, last_emergency_stop_time, alcohol_lock_active
    
    if alcohol_lock_active:
        add_log("Re-Arm Blocked: Alcohol level detected!", "danger")
        return JSONResponse(status_code=403, content={"status": "ALCOHOL_LOCKED", "msg": "Alcohol detected. Cannot arm."})

    with lock:
        emergency_halt_active = False
        alarm_level = 0
        restore_cooldown_until = time.time() + 4.0
        last_emergency_stop_time = 0.0
        telemetry["alarm_status"] = "OFF"
        telemetry["driver_condition"] = "SAFE"

    send_esp32_restore()
    add_log("System Restored & Re-Armed via Web Dashboard", "success")
    return JSONResponse(content={"status": "ARMED", "message": "Vehicle Re-Armed Successfully"})

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
    add_log("Emergency Halt Triggered manually via Dashboard", "danger")
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

                if action in ["arm", "restore", "rearm", "re-arm"]:
                    await api_restore()
                elif action in ["halt", "stop", "disconnect"]:
                    await api_halt()
        except Exception:
            pass

    rx_task = asyncio.create_task(receive_handler())
    try:
        while system_running:
            with lock:
                payload = {"telemetry": telemetry, "logs": system_logs[:12]}
            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(0.06)
    except (WebSocketDisconnect, asyncio.CancelledError):
        pass
    finally:
        rx_task.cancel()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")