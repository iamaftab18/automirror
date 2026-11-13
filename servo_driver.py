"""
Servo output for AutoMirror.

Tries, in order:
  1. Hardware PWM (rpi-hardware-pwm) - jitter-free, no daemon needed.
     Needs `dtoverlay=pwm-2chan` in /boot/firmware/config.txt (GPIO18 + GPIO19).
  2. pigpio - needs the pigpiod daemon running.
  3. Simulation - prints a warning, nothing moves (for testing on a PC).
"""
MIN_PULSE, MAX_PULSE = 600, 2400   # MG995 pulse range in microseconds
PERIOD_US = 20000                  # 50 Hz servo signal

# GPIO pin -> hardware PWM channel on the Pi 4 (pwm-2chan overlay)
HW_CHANNEL = {18: 0, 19: 1}


def angle_to_pulse(angle):
    return int(MIN_PULSE + (angle / 180.0) * (MAX_PULSE - MIN_PULSE))


class Servos:
    def __init__(self, pins):
        self.pins = list(pins)
        self.backend = "simulated"
        self._pwm = {}
        self._pi = None

        try:
            from rpi_hardware_pwm import HardwarePWM
            for pin in self.pins:
                pwm = HardwarePWM(pwm_channel=HW_CHANNEL[pin], hz=50, chip=0)
                pwm.start(0)
                self._pwm[pin] = pwm
            self.backend = "hardware PWM"
            return
        except Exception as exc:
            self._close_hw()
            hw_error = exc

        try:
            import pigpio
            pi = pigpio.pi()
            if not pi.connected:
                raise RuntimeError("pigpiod not running")
            self._pi = pi
            self.backend = "pigpio"
            return
        except Exception as exc:
            pig_error = exc

        print("[WARN] No servo output available - servos simulated.")
        print(f"       hardware PWM: {hw_error}")
        print(f"       pigpio:       {pig_error}")

    def set_angle(self, pin, angle):
        pulse = angle_to_pulse(angle)
        if pin in self._pwm:
            self._pwm[pin].change_duty_cycle(pulse / PERIOD_US * 100)
        elif self._pi:
            self._pi.set_servo_pulsewidth(pin, pulse)

    def off(self):
        """Stop the signal so the servos relax."""
        for pin in self.pins:
            if pin in self._pwm:
                self._pwm[pin].change_duty_cycle(0)
            elif self._pi:
                self._pi.set_servo_pulsewidth(pin, 0)

    def close(self):
        self.off()
        self._close_hw()
        if self._pi:
            self._pi.stop()
            self._pi = None

    def _close_hw(self):
        for pwm in self._pwm.values():
            try:
                pwm.stop()
            except Exception:
                pass
        self._pwm = {}
