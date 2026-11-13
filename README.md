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

## Install (Raspberry Pi OS 64-bit)
MediaPipe 0.10.x needs Python 3.9–3.12. Newer Pi OS ships Python 3.13, so use `uv` to make a Python 3.11 venv:
```bash
sudo apt update
sudo apt install -y git libopenblas-dev libgl1

# enable hardware PWM on GPIO18 + GPIO19 (jitter-free servos, no daemon)
echo "dtoverlay=pwm-2chan" | sudo tee -a /boot/firmware/config.txt
sudo reboot

curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env

uv venv --python 3.11 venv
source venv/bin/activate
uv pip install -r requirements.txt
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
- Servo output (`servo_driver.py`) uses hardware PWM first, then `pigpiod` if running, otherwise servos are simulated (e.g. testing on a PC) and the dashboard still works.
- Test the servos alone with `python servo.py` (or `servo.py pan` / `servo.py tilt`).
- Start with the mirror clamp loosened/no mirror to check directions first.
- Run at boot: create a systemd service that runs `venv/bin/python app.py` after `pigpiod`.
