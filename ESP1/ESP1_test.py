# esp1.py — Bed node: acts on "Motor ON"/"Motor OFF" strings from ESP3
# MicroPython (ESP32)
import network
import espnow
import machine
from machine import Pin
from time import sleep
import time
import utime

# --- Motor Definitions ---
class A4988Stepper:
    def __init__(self, step_pin, dir_pin, enable_pin=None, step_angle_deg=1.8):
        """
        step_pin      : GPIO number for STEP
        dir_pin       : GPIO number for DIR
        enable_pin    : GPIO number for ENABLE (optional, can be None)
        step_angle_deg: motor step angle in degrees (1.8° for most NEMA 17)
        """
        self.step = Pin(step_pin, Pin.OUT, value=0)
        self.dir  = Pin(dir_pin,  Pin.OUT, value=0)

        self.enable = None
        if enable_pin is not None:
            # A4988 ENABLE: LOW = enabled, HIGH = disabled
            self.enable = Pin(enable_pin, Pin.OUT, value=1)  # start disabled

        self.step_angle_deg = step_angle_deg
        # Full steps per revolution (e.g. 360 / 1.8 = 200)
        self.steps_per_rev = int(round(360.0 / self.step_angle_deg))

    def _enable_driver(self):
        if self.enable is not None:
            self.enable.value(0)   # LOW = enabled

    def _disable_driver(self):
        if self.enable is not None:
            self.enable.value(1)   # HIGH = disabled

    def enable(self):
        """Public helper to enable the driver (hold torque)."""
        self._enable_driver()

    def disable(self):
        """Public helper to disable the driver (motor off, no holding torque)."""
        self._disable_driver()

    def _step_once(self, delay_us):
        """
        Generate one step pulse with a given half-period delay (microseconds).
        Full step period is 2 * delay_us.
        """
        self.step.value(1)
        time.sleep_us(delay_us)
        self.step.value(0)
        time.sleep_us(delay_us)

    def move_degrees(self, angle_deg, rpm, dir_level, hold_enabled=False):
        """
        Move the motor by a given angle.

        angle_deg   : angle to move (in degrees, positive number)
                      1 step = 1.8 degrees by default.
        rpm         : motor speed in revolutions per minute (> 0)
        dir_level   : logic level to write to DIR pin (0 or 1)
                      You choose which level is "forward" based on wiring.
        hold_enabled: if False, disables driver after movement
        """

        if rpm <= 0:
            raise ValueError("rpm must be > 0")

        # Set direction explicitly from user input
        self.dir.value(1 if dir_level else 0)

        # Convert degrees to number of full steps
        angle_deg = abs(angle_deg)
        steps = int(round(angle_deg / self.step_angle_deg))
        print("Steps = " + str(steps))
        if steps == 0:
            return

        # Compute pulse timing from RPM
        # steps_per_sec = rpm * steps_per_rev / 60
        steps_per_sec = rpm * self.steps_per_rev / 60.0
        period_s = 1.0 / steps_per_sec
        half_period_us = int(period_s * 1_000_000 / 2)
        print("Half Period = " + str(half_period_us ) + " microseconds")

        # Clamp to a minimum delay to avoid insane speeds
        #if half_period_us < 2:
        #    half_period_us = 2

        self._enable_driver()

        for _ in range(steps):
            self._step_once(half_period_us)

        if not hold_enabled:
            self._disable_driver()

# ====== EDIT THESE ======
MAC_COORD = b"\x14\x2b\x2f\xae\xbe\x44"   # <-- paste ESP3 (door) STA MAC
CHANNEL   = 1
LED_PIN   = 13   # onboard LED on many ESP32 dev boards; change if needed
# ========================

MSG_ON  = "Motor ON"
MSG_OFF = "Motor OFF"

# ---- Radio / ESP-NOW ----
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(channel=CHANNEL)
ap.active(False)

print("[ESP1] MAC:", wlan.config('mac'))

e = espnow.ESPNow()
e.active(True)
e.add_peer(MAC_COORD)

# ---- Motor surrogate ----
#led = machine.Pin(LED_PIN, machine.Pin.OUT, value=0)
#def motor_on():
    #led.value(1)

#def motor_off():
    #led.value(0)

# --- motor setup 
STEP_PIN   = 25
DIR_PIN    = 26
ENABLE_PIN = 14  # or None if you didn't wire ENABLE

motor = A4988Stepper(
    step_pin=STEP_PIN,
    dir_pin=DIR_PIN,
    enable_pin=ENABLE_PIN,
    step_angle_deg=1.8  # 1 full step = 1.8°
    )
def motor_off():
    """Convenience function to turn the motor off (disable driver)."""
    motor.disable()

print("[ESP1] Ready. LED = motor.")

while True:
    try:
        mac, msg = e.recv(0)
        if mac and msg:
            try:
                s = msg.decode("utf-8")
            except:
                s = str(msg)

            if s == MSG_ON:
                # Loop until we see "Motor OFF"
                i = 1
                while True:

                    #motor.move_degrees(90, rpm=240)
                    #motor.spin_continuous(240, 1)
                    motor.move_degrees(90, 240, 1, hold_enabled=True)
                    sleep(0.2)
                    motor.move_degrees(90, 240, 0, hold_enabled=True)
                    sleep(0.2)
                    #motor.move_degrees(-90, rpm=120)
                    print("[ESP1] MOTOR ON " + str(i))
                    i += 1
                    # Check for a new message to stop the motor
                    mac, msg = e.recv(0)
                    if mac and msg:
                        try:
                            s = msg.decode("utf-8")
                        except:
                            s = str(msg2)

                        if s == MSG_OFF:
                            motor_off()
                            print("[ESP1] MOTOR OFF")
                            break

                    utime.sleep_ms(3)
            else:
                print("[ESP1] unhandled:", s)
                
            if s == MSG_OFF:
                motor_off()
                print("[ESP1] MOTOR OFF")
                break
    except OSError:
        pass

    utime.sleep_ms(3)
