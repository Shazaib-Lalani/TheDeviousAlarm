import network
import espnow
import machine
import time
from machine import Pin
from time import sleep
#import utime

# --- Motor Class Definition and Function Set Up---
class A4988Stepper:
    def __init__(self, step_pin, dir_pin, enable_pin=None, slp_pin=None, step_angle_deg=1.8):
        """
        step_pin      : GPIO number for STEP
        dir_pin       : GPIO number for DIR
        enable_pin    : GPIO number for ENABLE (can be None)
        slp_pin    : GPIO number for SLEEP (can be None)
        step_angle_deg: motor step angle in degrees (1.8°)
        """
        self.step = Pin(step_pin, Pin.OUT, value=0)
        self.dir  = Pin(dir_pin,  Pin.OUT, value=0)

        self.enable = None
        if enable_pin is not None:
            # A4988 ENABLE: LOW = enabled, HIGH = disabled
            self.enable = Pin(enable_pin, Pin.OUT, value=1)  # start disabled
            
        self.slp = None
        if slp_pin is not None:
            # A4988 ENABLE: LOW = enabled, HIGH = disabled
            self.slp = Pin(slp_pin, Pin.OUT, value=0)  # start asleep

        self.step_angle_deg = step_angle_deg
        # Full steps per revolution (e.g. 360 / 1.8 = 200)
        self.steps_per_rev = int(round(360.0 / self.step_angle_deg))

    def _enable_driver(self):
        if self.enable is not None:
            self.enable.value(0)   

    def _disable_driver(self):
        if self.enable is not None:
            self.enable.value(1)  

    def enable(self):
        """Public helper to enable the driver (hold torque)."""
        self._enable_driver()

    def disable(self):
        """Public helper to disable the driver (motor off, no holding torque)."""
        self._disable_driver()
        
    def _sleep_driver(self):
        if self.slp is not None:
            self.slp.value(0)   

    def _awake_driver(self):
        if self.slp is not None:
            self.slp.value(1)  

    def sleep(self):
        """Public helper to enable the driver (hold torque)."""
        self._sleep_driver()

    def awake(self):
        """Public helper to disable the driver (motor off, no holding torque)."""
        self._awake_driver()
        
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
        #print("Steps = " + str(steps))
        if steps == 0:
            return

        # Compute pulse timing from RPM
        steps_per_sec = rpm * self.steps_per_rev / 60.0
        period_s = 1.0 / steps_per_sec
        half_period_us = int(period_s * 1_000_000 / 2)
        #print("Half Period = " + str(half_period_us ) + " microseconds")

        self._enable_driver()

        for _ in range(steps):
            self._step_once(half_period_us)

        if not hold_enabled:
            self._disable_driver()
            
    def home_position(self, lmt, rpm, dir_level):
        print("in function")
        while lmt.value() > 0:
            print("in while loop")
            print(lmt.value())
            self.move_degrees(1.8, rpm, dir_level,hold_enabled=True)        

# --- motor setup 
STEP_PIN   = 25
DIR_PIN    = 26
ENABLE_PIN = 14
SLEEP_PIN  = 32

motor = A4988Stepper(
    step_pin=STEP_PIN,
    dir_pin=DIR_PIN,
    enable_pin=ENABLE_PIN,
    slp_pin=SLEEP_PIN,
    step_angle_deg=1.8  # 1 full step = 1.8°
    )

# ====== Mac Address Set Up ======
#MAC_COORD = b"\x14\x2b\x2f\xae\xbe\x44"   
#CHANNEL   = 1

# ========Defining what Messages Will Be Received================
ALM_1 = "ALARM 1"
ALM_2 = "ALARM 2"
ALM_3 = "ALARM 3"
PS = "PAUSE"
SD = "SHUTDOWN"

# ======== Radio / ESP-NOW Initialization ================
#wlan = network.WLAN(network.STA_IF)
#wlan.active(True)

#ap = network.WLAN(network.AP_IF)
#ap.active(True)
#ap.config(channel=CHANNEL)
#ap.active(False)

#print("[ESP1] MAC:", wlan.config('mac'))

#e = espnow.ESPNow()
#e.active(True)
#e.add_peer(MAC_COORD)

# ======= Solenoid Valve Initialization ==========
fet = Pin(15, Pin.OUT)
fet.value(0)

# ======= Limit Switch Initialization =========
limitSwitch = Pin(4, Pin.IN, Pin.PULL_UP)
print(limitSwitch.value())

# ===== Homing Motor ========
motor.enable()
motor.awake()
print("gets here")
motor.home_position(limitSwitch, 15, 0)
motor.sleep()