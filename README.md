# AutoMirror – Face-Tracking Rear-View Mirror (Raspberry Pi 4B)

USB camera + MediaPipe Face Mesh drive two MG995 servos (pan / tilt) so the mirror follows the driver's eyes.
Flask dashboard with live video, face mesh overlay, angle display and Start / Stop.

## Files
```
app.py
requirements.txt
templates/dashboard.html
```

## Wiring
| Servo | Signal pin | Power |
|---|---|---|
| Tilt (Servo 1, up/down) | GPIO18 (pin 12) | 6 V from buck converter |
| Pan (Servo 2, left/right) | GPIO19 (pin 35) | 6 V from buck converter |

- Never power servos from the Pi 5 V pin.
- Connect the buck converter **GND to a Pi GND pin**.
- Only one servo fitted? Set `USE_TILT = False` in `app.py`.

## Install (Raspberry Pi OS 64-bit, Python 3.9–3.11)
```bash
sudo apt update
sudo apt install -y pigpio python3-venv libatlas-base-dev
sudo systemctl enable --now pigpiod

python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Run
```bash
python app.py
```
Open `http://<PI_IP>:5000` from any device on the network (find the IP with `hostname -I`).

## Usage
- **Start** – mirror slowly follows your face position.
- **Stop** – servos stop and hold the current angle (locked).
- Press **Start** again to resume.
- If no face is detected, the mirror stays where it is.

## Tuning (top of `app.py`)
| Setting | Meaning |
|---|---|
| `MAX_SPEED` | Servo speed in °/s (lower = slower) |
| `DEADZONE` | Ignore tiny changes (removes jitter) |
| `SMOOTHING` | Face target smoothing (lower = smoother) |
| `MIRROR_RATIO` | 0.5 = mirror turns half the head angle (mirror law) |
| `INVERT_PAN` / `INVERT_TILT` | Flip direction if the mirror moves the wrong way |
| `PAN_LIMIT` / `TILT_LIMIT` | Max travel from centre (3D model is tested to ±90° pan, ±35° tilt) |
| `HFOV` / `VFOV` | Your camera's field of view |
| `MIN_PULSE` / `MAX_PULSE` | Servo pulse range (adjust if your MG995 clone differs) |

## Notes
- Centre both servos (90°) before fitting horns; at startup the servos move to 90°.
- The camera sits in the fixed enclosure, so the system uses face position directly (no feedback from the mirror).
- Without `pigpiod` (e.g. testing on a PC) servos are simulated and the dashboard still works.
- Start with the mirror clamp loosened/no mirror to check directions first.
- Run at boot: create a systemd service that runs `venv/bin/python app.py` after `pigpiod`.
