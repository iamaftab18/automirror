"""
Servo test - sweeps the pan and tilt servos in a loop.

Usage:
    python servo.py           # test both servos
    python servo.py pan       # test only pan  (GPIO19)
    python servo.py tilt      # test only tilt (GPIO18)

Stop with Ctrl+C (or `pkill -f servo.py`) - servos are switched off on exit.
Uses hardware PWM (dtoverlay=pwm-2chan) or pigpio - see servo_driver.py.
"""
import signal
import sys
import time

from servo_driver import Servos, angle_to_pulse

PAN_PIN = 19    # Servo 2 (left/right)
TILT_PIN = 18   # Servo 1 (up/down)

LOW, HIGH = 60, 120                # sweep range in degrees (90 = centre)
STEP = 2                           # degrees per step
DELAY = 0.03                       # seconds per step (bigger = slower)


def main():
    which = sys.argv[1].lower() if len(sys.argv) > 1 else "both"
    pins = {"pan": [PAN_PIN], "tilt": [TILT_PIN], "both": [PAN_PIN, TILT_PIN]}.get(which)
    if pins is None:
        sys.exit("Usage: python servo.py [pan|tilt|both]")

    servos = Servos(pins)
    if servos.backend == "simulated":
        sys.exit("No servo output - see the warnings above.")

    def cleanup(*_):
        servos.close()   # stop pulses -> servo relaxes
        print("\nServos off. Bye.")
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    def move(angle):
        for p in pins:
            servos.set_angle(p, angle)
        print(f"\rAngle: {angle:5.1f} deg   pulse: {angle_to_pulse(angle)} us ", end="", flush=True)

    print(f"Testing {which} servo(s) on GPIO {pins} using {servos.backend}. Ctrl+C to stop.")
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
