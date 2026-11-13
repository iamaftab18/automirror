"""
Servo test - sweeps the pan and tilt servos in a loop.

Usage:
    python servo.py           # test both servos
    python servo.py pan       # test only pan  (GPIO19)
    python servo.py tilt      # test only tilt (GPIO18)

Stop with Ctrl+C (or `pkill -f servo.py`) - servos are switched off on exit.
Needs pigpiod running:  sudo systemctl start pigpiod   (or: sudo pigpiod)
"""
import signal
import sys
import time

import pigpio

PAN_PIN = 19    # Servo 2 (left/right)
TILT_PIN = 18   # Servo 1 (up/down)

MIN_PULSE, MAX_PULSE = 600, 2400   # same as app.py
LOW, HIGH = 60, 120                # sweep range in degrees (90 = centre)
STEP = 2                           # degrees per step
DELAY = 0.03                       # seconds per step (bigger = slower)


def angle_to_pulse(angle):
    return int(MIN_PULSE + (angle / 180.0) * (MAX_PULSE - MIN_PULSE))


def main():
    which = sys.argv[1].lower() if len(sys.argv) > 1 else "both"
    pins = {"pan": [PAN_PIN], "tilt": [TILT_PIN], "both": [PAN_PIN, TILT_PIN]}.get(which)
    if pins is None:
        sys.exit("Usage: python servo.py [pan|tilt|both]")

    pi = pigpio.pi()
    if not pi.connected:
        sys.exit("pigpiod is not running. Start it with: sudo systemctl start pigpiod")

    def cleanup(*_):
        for p in pins:
            pi.set_servo_pulsewidth(p, 0)   # stop pulses -> servo relaxes
        pi.stop()
        print("\nServos off. Bye.")
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    def move(angle):
        for p in pins:
            pi.set_servo_pulsewidth(p, angle_to_pulse(angle))
        print(f"\rAngle: {angle:5.1f} deg   pulse: {angle_to_pulse(angle)} us ", end="", flush=True)

    print(f"Testing {which} servo(s) on GPIO {pins}. Ctrl+C to stop.")
    move(90)
    time.sleep(1)

    while True:
        for a in range(90, HIGH + 1, STEP):       # centre -> high
            move(a); time.sleep(DELAY)
        for a in range(HIGH, LOW - 1, -STEP):     # high -> low
            move(a); time.sleep(DELAY)
        for a in range(LOW, 91, STEP):            # low -> centre
            move(a); time.sleep(DELAY)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
