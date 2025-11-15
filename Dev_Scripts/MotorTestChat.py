# One fast 360° rotation with A4988 + ESP32 (MicroPython)
# - Full steps (fastest). Change to 2/4/8/16 if you need smoother motion.
# - Short accel/decel ramp to avoid stalls.
#
# Wiring defaults (Adafruit ESP32 Feather V2 friendly):
#   STEP -> IO25, DIR -> IO33, EN -> IO32 (active LOW)
#   MS1 -> IO14, MS2 -> IO27, MS3 -> IO26
#
# Power & safety:
#   - Set A4988 current limit first.
#   - Put a ≥47 µF electrolytic across VMOT–GND near the driver.
#   - Connect motor BEFORE power.

from machine import Pin
from time import sleep_us, sleep_ms, ticks_us, ticks_diff

# ---------------- Pin config (edit if needed) ----------------
PIN_STEP = 25
PIN_DIR  = 33
PIN_EN   = 32
PIN_MS1  = 14
PIN_MS2  = 27
PIN_MS3  = 26
# -------------------------------------------------------------

FULL_STEPS_PER_REV = 200  # most NEMA17 are 1.8°/step
MIN_STEP_PULSE_US  = 2    # A4988 min ~1 µs; use 2 µs for margin
MIN_STEP_SPACE_US  = 2

# Choose aggressive but safe-ish top speed:
# For many NEMA17+A4988 setups, ~400–800 RPM is near the practical ceiling.
TARGET_RPM   = 600       # bump up/down if your rig can/can’t handle it
ACCEL_FRAC   = 0.1       # 10% accel + 10% decel of total steps (short ramp)
MICROSTEP    = 1         # 1, 2, 4, 8, 16


class A4988:
    _ms_map = {
        1:(0,0,0),
        2:(1,0,0),
        4:(0,1,0),
        8:(1,1,0),
        16:(1,1,1),
    }

    def __init__(self):
        self.step = Pin(PIN_STEP, Pin.OUT, value=0)
        self.dir  = Pin(PIN_DIR,  Pin.OUT, value=0)
        self.en   = Pin(PIN_EN,   Pin.OUT, value=1)  # 1=disabled
        self.ms1  = Pin(PIN_MS1,  Pin.OUT, value=0)
        self.ms2  = Pin(PIN_MS2,  Pin.OUT, value=0)
        self.ms3  = Pin(PIN_MS3,  Pin.OUT, value=0)
        self.microstep = 1
        self.set_microstep(MICROSTEP)

    def enable(self):
        self.en.value(0)
        sleep_ms(2)

    def disable(self):
        self.en.value(1)

    def set_dir(self, cw=True):
        self.dir.value(0 if cw else 1)

    def set_microstep(self, m):
        ms = self._ms_map.get(m)
        if not ms:
            raise ValueError("Microstep must be 1,2,4,8,16")
        self.microstep = m
        self.ms1.value(ms[0]); self.ms2.value(ms[1]); self.ms3.value(ms[2])
        sleep_ms(1)

    def _half_delay_from_rpm(self, rpm):
        steps_per_rev = FULL_STEPS_PER_REV * self.microstep
        steps_per_sec = (steps_per_rev * rpm) / 60.0
        period_us = 1_000_000.0 / steps_per_sec
        return max(period_us/2.0 - MIN_STEP_PULSE_US, 0)

    def _do_step(self, half_delay_us):
        self.step.value(1)
        sleep_us(MIN_STEP_PULSE_US)
        self.step.value(0)
        sleep_us(int(max(half_delay_us, MIN_STEP_SPACE_US)))

    def move_steps(self, steps, rpm, accel_frac=0.1):
        if steps == 0: return
        total = abs(int(steps))
        self.set_dir(steps > 0)
        self.enable()

        a = max(0.0, min(0.45, float(accel_frac)))
        n_accel = int(total * a)
        n_decel = int(total * a)
        n_flat  = max(0, total - n_accel - n_decel)

        target_hd = self._half_delay_from_rpm(rpm)
        start_hd  = target_hd * 5.0  # quick ramp from ~5x slower

        def ramp(n, start_d, end_d):
            if n <= 0: return []
            step = (end_d - start_d) / n
            return [start_d + i*step for i in range(n)]

        accel = ramp(n_accel, start_hd, target_hd)
        flat  = [target_hd]*n_flat
        decel = ramp(n_decel, target_hd, start_hd)

        try:
            for d in accel: self._do_step(d)
            for d in flat:  self._do_step(d)
            for d in decel: self._do_step(d)
        finally:
            self.disable()


def rotate_360_fast(clockwise=True):
    drv = A4988()
    drv.set_microstep(MICROSTEP)          # keep at 1 for highest practical speed
    steps_per_rev = FULL_STEPS_PER_REV * MICROSTEP
    drv.move_steps(steps_per_rev if clockwise else -steps_per_rev,
                   rpm=TARGET_RPM,
                   accel_frac=ACCEL_FRAC)


# ------------------- Run once -------------------
if __name__ == "__main__":
    # One full revolution at near max speed
    for i in range(10):
        rotate_360_fast(clockwise=True)
    print("done")