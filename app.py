"""
AutoMirror - face-tracking rear-view mirror for Raspberry Pi 4B.

Camera -> MediaPipe Face Mesh -> slow, smoothed servo movement (pan + tilt).
Dashboard: http://<pi-ip>:5000
"""
import threading
import time

import cv2
import mediapipe as mp
from flask import Flask, Response, jsonify, render_template

from servo_driver import Servos

# ----------------------------- CONFIGURATION ------------------------------
CAMERA_INDEX = 0
FRAME_W, FRAME_H = 640, 480

PAN_PIN = 19          # Servo 2 (left/right)  - hardware PWM pin
TILT_PIN = 18         # Servo 1 (up/down)     - hardware PWM pin
USE_TILT = True       # set False if only one servo (pan) is fitted

CENTER = 90.0         # servo angle where the mirror looks straight back
PAN_LIMIT = 30.0      # max +/- degrees from centre
TILT_LIMIT = 20.0

# Mirror law: turning a mirror by X deg moves the reflected view by 2X deg,
# so the mirror only needs to turn about half the angle the head moved.
HFOV, VFOV = 60.0, 45.0   # approx. camera field of view (degrees)
MIRROR_RATIO = 0.5
INVERT_PAN = False        # flip if the mirror turns the wrong way left/right
INVERT_TILT = False       # flip if the mirror turns the wrong way up/down

MAX_SPEED = 10.0      # servo speed limit in deg/second  (lower = slower)
DEADZONE = 1.5        # ignore changes smaller than this (deg) - stops jitter
SMOOTHING = 0.15      # 0..1, lower = smoother face target

JPEG_QUALITY = 70        # (servo pulse range: MIN_PULSE/MAX_PULSE in servo_driver.py)
# --------------------------------------------------------------------------

servos = Servos([PAN_PIN, TILT_PIN] if USE_TILT else [PAN_PIN])
print(f"[INFO] Servo output: {servos.backend}")


def set_servo(pin, angle):
    servos.set_angle(pin, angle)


class Mirror:
    def __init__(self):
        self.pan = self.tilt = CENTER
        self.target_pan = self.target_tilt = CENTER
        self.running = False
        self.face = False
        self.fps = 0.0
        self.offset = (0.0, 0.0)
        self.jpeg = None
        self.lock = threading.Lock()
        set_servo(PAN_PIN, self.pan)
        if USE_TILT:
            set_servo(TILT_PIN, self.tilt)

    def _step(self, current, target, dt):
        diff = target - current
        if abs(diff) < DEADZONE:
            return current
        step = min(abs(diff), MAX_SPEED * dt)
        return current + step if diff > 0 else current - step

    def update(self, dt):
        """Move servos slowly toward target (only while running)."""
        if not self.running:
            return  # servo keeps last pulse -> holds position
        new_pan = self._step(self.pan, self.target_pan, dt)
        new_tilt = self._step(self.tilt, self.target_tilt, dt)
        if new_pan != self.pan:
            self.pan = new_pan
            set_servo(PAN_PIN, self.pan)
        if USE_TILT and new_tilt != self.tilt:
            self.tilt = new_tilt
            set_servo(TILT_PIN, self.tilt)

    def set_face(self, nx, ny):
        """nx, ny: face position, -1..1 from frame centre."""
        self.offset = (nx, ny)
        pan = nx * (HFOV / 2) * MIRROR_RATIO * (-1 if INVERT_PAN else 1)
        tilt = ny * (VFOV / 2) * MIRROR_RATIO * (-1 if INVERT_TILT else 1)
        self.target_pan = max(CENTER - PAN_LIMIT, min(CENTER + PAN_LIMIT, CENTER + pan))
        self.target_tilt = max(CENTER - TILT_LIMIT, min(CENTER + TILT_LIMIT, CENTER + tilt))


mirror = Mirror()
mp_face = mp.solutions.face_mesh
mp_draw = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles


def vision_loop():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    face_mesh = mp_face.FaceMesh(max_num_faces=1, refine_landmarks=False,
                                 min_detection_confidence=0.5,
                                 min_tracking_confidence=0.5)
    last = time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        frame = cv2.flip(frame, 1)  # mirror view, natural for the driver
        h, w = frame.shape[:2]
        result = face_mesh.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        now = time.time()
        dt, last = now - last, now
        mirror.fps = 0.9 * mirror.fps + 0.1 * (1 / dt) if dt > 0 else mirror.fps

        if result.multi_face_landmarks:
            lm = result.multi_face_landmarks[0]
            mirror.face = True
            mp_draw.draw_landmarks(
                frame, lm, mp_face.FACEMESH_TESSELATION, None,
                mp_styles.get_default_face_mesh_tesselation_style())
            mp_draw.draw_landmarks(
                frame, lm, mp_face.FACEMESH_CONTOURS, None,
                mp_styles.get_default_face_mesh_contours_style())
            # eye midpoint = reference point for the driver's eyes
            a, b = lm.landmark[33], lm.landmark[263]
            ex, ey = (a.x + b.x) / 2, (a.y + b.y) / 2
            raw_x, raw_y = (ex - 0.5) * 2, (ey - 0.5) * 2
            ox, oy = mirror.offset
            mirror.set_face(ox + SMOOTHING * (raw_x - ox), oy + SMOOTHING * (raw_y - oy))
            cv2.circle(frame, (int(ex * w), int(ey * h)), 6, (0, 255, 255), -1)
        else:
            mirror.face = False  # keep the last target, mirror stays put

        # crosshair
        cv2.line(frame, (w // 2 - 15, h // 2), (w // 2 + 15, h // 2), (255, 255, 255), 1)
        cv2.line(frame, (w // 2, h // 2 - 15), (w // 2, h // 2 + 15), (255, 255, 255), 1)

        mirror.update(dt)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            with mirror.lock:
                mirror.jpeg = buf.tobytes()


app = Flask(__name__)


def mjpeg():
    while True:
        with mirror.lock:
            jpg = mirror.jpeg
        if jpg:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        time.sleep(0.04)


@app.route("/")
def index():
    return render_template("dashboard.html")


@app.route("/video_feed")
def video_feed():
    return Response(mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/start", methods=["POST"])
def start():
    mirror.running = True
    return jsonify(running=True)


@app.route("/stop", methods=["POST"])
def stop():
    mirror.running = False  # servos hold the current angle
    return jsonify(running=False)


@app.route("/status")
def status():
    return jsonify(running=mirror.running, face=mirror.face,
                   pan=round(mirror.pan, 1), tilt=round(mirror.tilt, 1),
                   target_pan=round(mirror.target_pan, 1),
                   target_tilt=round(mirror.target_tilt, 1),
                   fps=round(mirror.fps, 1),
                   offset_x=round(mirror.offset[0], 2),
                   offset_y=round(mirror.offset[1], 2),
                   pan_limit=PAN_LIMIT, tilt_limit=TILT_LIMIT, center=CENTER)


if __name__ == "__main__":
    threading.Thread(target=vision_loop, daemon=True).start()
    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        servos.close()  # stop sending pulses on exit (servo relaxes)
