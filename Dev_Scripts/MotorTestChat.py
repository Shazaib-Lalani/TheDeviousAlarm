from machine import Pin
import time

class A4988Stepper:
    def __init__(self, step_pin, dir_pin, enable_pin=None, steps_per_rev=200):
        """
        step_pin   : GPIO number for STEP
        dir_pin    : GPIO number for DIR
        enable_pin : GPIO number for ENABLE (optional, can be None)
        steps_per_rev : full steps per revolution (200 for 1.8° NEMA 17)
        """

        self.step = Pin(step_pin, Pin.OUT, value=0)
        self.dir  = Pin(dir_pin,  Pin.OUT, value=0)

        self.enable = None
        if enable_pin is not None:
            # On A4988, ENABLE is usually:
            #   LOW  = enabled
            #   HIGH = disabled
            self.enable = Pin(enable_pin, Pin.OUT, value=1)  # start disabled

        self.steps_per_rev = steps_per_rev

    def _enable_driver(self):
        if self.enable is not None:
            self.enable.value(0)   # LOW = enabled

    def _disable_driver(self):
        if self.enable is not None:
            self.enable.value(1)   # HIGH = disabled

    def _step_once(self, delay_us):
        """
        Generate one step pulse with a given half-period delay (microseconds).
        Full step period is 2 * delay_us.
        """
        self.step.value(1)
        time.sleep_us(delay_us)
        self.step.value(0)
        time.sleep_us(delay_us)

    def move_steps(self, steps, rpm=60, direction=None, hold_enabled=False):
        """
        Move a given number of full steps at a given speed (rpm).

        steps       : signed integer number of steps
                      (positive = one direction, negative = opposite)
        rpm         : speed in revolutions per minute
        direction   : optional explicit direction (+1 or -1).
                      If None, direction is taken from the sign of 'steps'.
        hold_enabled: if False, disables driver after movement (if ENABLE pin given)
        """

        if steps == 0:
            return

        # Determine direction
        if direction is not None:
            dir_val = 1 if direction > 0 else 0
        else:
            dir_val = 1 if steps > 0 else 0

        self.dir.value(dir_val)

        step_count = abs(int(steps))

        # Convert RPM to delay between step edges
        # Steps per second = rpm * steps_per_rev / 60
        # Period per step = 1 / steps_per_second
        # Use half-period for high and low times.
        if rpm <= 0:
            raise ValueError("rpm must be > 0")

        steps_per_sec = rpm * self.steps_per_rev / 60.0
        period_s = 1.0 / steps_per_sec
        half_period_us = int(period_s * 1_000_000 / 2)

        if half_period_us < 2:
            # Prevent zero or crazy high speed
            half_period_us = 2

        self._enable_driver()

        for _ in range(step_count):
            self._step_once(half_period_us)

        if not hold_enabled:
            self._disable_driver()

    def move_degrees(self, angle_deg, rpm=60, hold_enabled=False):
        """
        Rotate the motor by a specified angle in degrees at a given rpm.

        angle_deg   : signed angle in degrees (+ = one direction, - = opposite)
        rpm         : speed in RPM
        hold_enabled: if False, disables driver after movement
        """

        # steps = angle / 360 * steps_per_rev
        steps_f = angle_deg / 360.0 * self.steps_per_rev
        steps = int(round(steps_f))

        self.move_steps(steps, rpm=rpm, hold_enabled=hold_enabled)

    def move_revolutions(self, revolutions, rpm=60, hold_enabled=False):
        """
        Rotate the motor by a specified number of revolutions.

        revolutions : signed number of revs (+ or -)
        rpm         : speed in RPM
        """
        steps_f = revolutions * self.steps_per_rev
        steps = int(round(steps_f))
        self.move_steps(steps, rpm=rpm, hold_enabled=hold_enabled)

    def spin_continuous(self, rpm=60, direction=1):
        """
        Spin continuously at a given RPM until interrupted (Ctrl+C).
        Blocking call.
        """
        if rpm <= 0:
            raise ValueError("rpm must be > 0")

        dir_val = 1 if direction > 0 else 0
        self.dir.value(dir_val)

        steps_per_sec = rpm * self.steps_per_rev / 60.0
        period_s = 1.0 / steps_per_sec
        half_period_us = int(period_s * 1_000_000 / 2)

        if half_period_us < 2:
            half_period_us = 2

        self._enable_driver()
        try:
            while True:
                self._step_once(half_period_us)
        except KeyboardInterrupt:
            # Stop on Ctrl+C
            self._disable_driver()
            print("Spin stopped")


# ----------------------------------------------------------------------
# USER CONFIGURATION SECTION
# ----------------------------------------------------------------------
# Change these pin numbers to match whatever pins you want to use
# on the Adafruit ESP32 Feather V2.
STEP_PIN   = 25   # example: GPIO12
DIR_PIN    = 26   # example: GPIO13
ENABLE_PIN = 14   # example: GPIO14 (optional; set to None if not wired)
SLEEP_PIN = 14
# Full-step NEMA 17 is usually 200 steps / rev
STEPS_PER_REV = 200


motor = A4988Stepper(
    step_pin=STEP_PIN,
    dir_pin=DIR_PIN,
    enable_pin=ENABLE_PIN,
    steps_per_rev=STEPS_PER_REV
)

# ----------------------------------------------------------------------
# EXAMPLE USAGE
# ----------------------------------------------------------------------
if __name__ == "__main__":
    # Example 1: Move +90 degrees at 30 RPM
    print("Rotating +90 degrees at 30 RPM...")
    motor.move_degrees(90, rpm=30)

    time.sleep(1)

    # Example 2: Move -180 degrees at 60 RPM (reverse)
    print("Rotating -180 degrees at 60 RPM...")
    motor.move_degrees(-180, rpm=60)

    time.sleep(1)

    # Example 3: One full revolution at 120 RPM
    while True:
        print("1 full revolution at 120 RPM...")
        motor.move_revolutions(1, rpm=120)

        time.sleep(1)

    # Example 4: Continuous spin (Ctrl+C to stop)
    # print("Spinning continuously at 60 RPM (Ctrl+C to stop)...")
    # motor.spin_continuous(rpm=60, direction=1)
