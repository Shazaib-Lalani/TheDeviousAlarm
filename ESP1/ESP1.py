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
    
    def _rpm_to_delay_us(self, rpm):
        """
        Convert RPM to half-period delay in microseconds for one step pulse.
        """
        if rpm <= 0:
            rpm = 1  # avoid divide by zero
        steps_per_sec = rpm * self.steps_per_rev / 60.0
        period_s = 1.0 / steps_per_sec
        half_period_us = int(period_s * 1_000_000 / 2)

        # Clamp to something sane so we don't go crazy-fast
        if half_period_us < 2:
            half_period_us = 2

        return half_period_us
        
    def _step_once(self, delay_us):
        """
        Generate one step pulse with a given half-period delay (microseconds).
        Full step period is 2 * delay_us.
        """
        self.step.value(1)
        time.sleep_us(delay_us)
        self.step.value(0)
        time.sleep_us(delay_us)

    def move_degrees(self, angle_deg, rpm_target, dir_level,
                     hold_enabled=False, accel_frac=0.4, min_rpm=5):
        """
        Move the motor by a given angle with acceleration only (no decel).

        angle_deg   : positive angle in degrees
        rpm_target  : target cruising speed
        dir_level   : 0 or 1 for DIR pin
        hold_enabled: keep driver enabled at the end if True
        accel_frac  : fraction of steps used to accelerate up to rpm_target
        min_rpm     : starting rpm at the beginning of the move
        """
        if rpm_target <= 0:
            raise ValueError("rpm must be > 0")

        # Set direction
        self.dir.value(1 if dir_level else 0)

        angle_deg = abs(angle_deg)
        total_steps = int(round(angle_deg / self.step_angle_deg))
        if total_steps <= 0:
            return

        # Steps used for acceleration (rest at full speed)
        accel_steps = int(total_steps * accel_frac)
        if accel_steps < 1:
            accel_steps = 1
        if accel_steps > total_steps:
            accel_steps = total_steps

        self._enable_driver()

        for i in range(total_steps):
            # Accelerate from min_rpm to rpm_target over accel_steps
            if i < accel_steps:
                phase = i / max(1, accel_steps - 1)
                rpm = min_rpm + (rpm_target - min_rpm) * phase
            else:
                # After accel phase, stay at full rpm_target (no decel)
                rpm = rpm_target

            delay_us = self._rpm_to_delay_us(rpm)
            self._step_once(delay_us)

        if not hold_enabled:
            self._disable_driver()
            
    def home_position(self, lmt, rpm, dir_level):
        ind = 1
        while lmt.value() == 1:
            if ind == 1:
                self.move_degrees(1.8 * 2, rpm, dir_level, hold_enabled=True, accel_frac=0.6, min_rpm=3)
            else:
                self.move_degrees(1.8 * 2, rpm, dir_level, hold_enabled=True, accel_frac=0, min_rpm=rpm)
            ind += 1
              
def sync_to_latest(max_reads=300, max_ms=20):
    """
    Bounded catch-up: discard backlog without getting stuck if messages arrive
    continuously.

    Reads and discards up to max_reads packets OR for up to max_ms milliseconds,
    then stops. Safe even if the sender is spamming.
    """
    start = time.ticks_ms()
    reads = 0

    while reads < max_reads and time.ticks_diff(time.ticks_ms(), start) < max_ms:
        pkt = e.recv(0)   # non-blocking
        if not pkt:
            break         # queue empty right now
        reads += 1


def get_next_non_ping():
    """
    AFTER you've flushed, wait for the next message that is not 'PING'.
    Ignores PINGs that arrive going forward.
    Returns the first non-PING message string.
    """
    while True:
        pkt = e.recv(0)  # non-blocking
        if not pkt:
            time.sleep(0.005)   # tiny yield so you don't spin at 100% CPU
            continue

        mac, msg = pkt
        if not (mac and msg):
            continue

        try:
            s = msg.decode("utf-8").strip()
        except:
            s = str(msg).strip()

        if s == "PING":
            continue  # keep looping until NOT a ping

        break  # got a non-PING message -> use it
    
    return s
                       
def check_interrupt(ind):
    sync_to_latest(max_reads=300, max_ms=100)

    # Now wait for the next non-ping message that arrives AFTER this point
    s = get_next_non_ping()

    # 2) outside the loop: handle pause/shutdown once
    if s == PS:
        print("==== Pause from Interrupt ====")
        fet.value(0)
        if ind == 1:
            motor.home_position(limitSwitch, 10, 0)
        motor.disable()
        return PS  # return the message (or return True if you prefer)

    if s == SD:
        print("==== Shut Down from Interrupt ====")
        fet.value(0)
        motor.home_position(limitSwitch, 10, 0)
        motor.disable()
        raise SystemExit

    # 3) otherwise just return the non-PING message (e.g. ALARM 1/2/3)
    return s



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
MAC_COORD = b"\x14\x2b\x2f\xae\xbe\x44"   
CHANNEL   = 1

# ========Defining what Messages Will Be Received================
ALM_1 = "ALARM 1"
ALM_2 = "ALARM 2"
ALM_3 = "ALARM 3"
PS = "PAUSE"
SD = "SHUTDOWN"

# ======== Radio / ESP-NOW Initialization ================
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

# ======= Solenoid Valve Initialization ==========
fet = Pin(15, Pin.OUT)
fet.value(0)

# ======= Limit Switch Initialization =========
limitSwitch = Pin(4, Pin.IN, Pin.PULL_UP)

# ===== Homing Motor ========
motor.enable()
motor.awake()
motor.home_position(limitSwitch, 10, 0)
motor.disable()

# logic loop
ind = 1
while True:
    try:
        sync_to_latest(max_reads=300, max_ms=100)
        
        s = get_next_non_ping()
        
        print("s = " + s)

        if s == ALM_1 or s == ALM_2:
            ind = 1
            print("==== ALARM 1 or 2 ====")
            motor.enable()
            motor.awake()

            motor.move_degrees(180, 100, 1, hold_enabled=True, accel_frac=0.4, min_rpm=10)
            
            s = check_interrupt(ind)
            
            time.sleep(0.02)
            motor.disable()
            time.sleep(1)
            motor.enable()
            
            s = check_interrupt(ind)
            
            motor.home_position(limitSwitch, 10, 0)
            time.sleep(0.2)

        elif s == ALM_3:
            ind = 1
            print("==== ALARM 3 ====")
            motor.enable()
            motor.awake()

            motor.move_degrees(180, 100, 1, hold_enabled=True, accel_frac=0.4, min_rpm=10)
            
            s = check_interrupt(ind)
            
            time.sleep(0.02)
            motor.disable()
            time.sleep(1)
            motor.enable()

            s = check_interrupt(ind)

            motor.home_position(limitSwitch, 10, 0)

            s = check_interrupt(ind)
            
            time.sleep(0.4)
            fet.value(1)
            time.sleep(0.4)
            fet.value(0)
            time.sleep(0.4)

        elif s == PS:
            print("==== Pause ====")
            fet.value(0)
            if ind == 1:
                motor.home_position(limitSwitch, 10, 0)
            motor.disable()
            ind += 1

        elif s == SD:
            print("==== Shut Down ====")
            fet.value(0)
            motor.home_position(limitSwitch, 10, 0)
            motor.disable()
            break

    except SystemExit:
        break
    except Exception as err:
        print("Error In Message Receiving:", err)
        time.sleep(0.1)
