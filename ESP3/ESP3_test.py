# esp3.py — Coordinator: sends "Motor ON"/"Motor OFF" strings via ESP-NOW
# MicroPython (ESP32) — MAX4466 mic on ADC, LSM6DS0 IMU on I2C
import network, espnow, machine, utime, ustruct

# ====== EDIT THESE ======
MAC_ESP1 = b"\xf4\x65\x0b\x30\x96\xb8"   # <-- paste ESP1 (bed) STA MAC
CHANNEL  = 1

# Mic (MAX4466) on a quiet ADC1 pin
MIC_PIN  = 40
MIC_ATTEN = machine.ADC.ATTN_11DB

# I2C for LSM6DS0 (Qwiic)
I2C_SDA, I2C_SCL, I2C_FREQ = 22, 20, 400000

# Optional buttons for manual test (active-low to GND)
BTN_ON   = 33   # press => "Motor ON"
BTN_OFF  = 32   # press => "Motor OFF"
# ========================

MSG_ON  = "Motor ON"
MSG_OFF = "Motor OFF"

# ---- Radio / ESP-NOW ----
wlan = network.WLAN(network.STA_IF); wlan.active(True)
ap = network.WLAN(network.AP_IF); ap.active(True); ap.config(channel=CHANNEL); ap.active(False)
print("[ESP3] MAC:", wlan.config('mac'))
e = espnow.ESPNow(); e.active(True); e.add_peer(MAC_ESP1)

def send_str(s):
    try:
        e.send(MAC_ESP1, s.encode("utf-8"))
        print("[ESP3] sent ->", s)
    except Exception as err:
        print("[ESP3] send err:", err)

# ---- Mic (MAX4466) ----
adc = machine.ADC(machine.Pin(MIC_PIN)); adc.atten(MIC_ATTEN)

def mic_dev(n=64):
    s=0
    for _ in range(n): s += adc.read()
    avg = s//n
    d=0
    for _ in range(n): d += abs(adc.read() - avg)
    return d//n

MIC_BASELINE_SAMPLES = 250
MIC_MARGIN           = 180   # raise if too sensitive, lower if not sensitive
MIC_HOLD_MS          = 600   # must be loud this long to trigger

# ---- IMU (LSM6DS0) minimal ----
class LSM6DS0:
    WHO_AM_I  = 0x0F
    CTRL_REG1_G  = 0x10
    CTRL_REG6_XL = 0x20
    OUT_X_L_G    = 0x18
    OUT_X_L_XL   = 0x28
    def __init__(self, i2c):
        self.i2c = i2c; self.addr=None
        for a in (0x6A, 0x6B):
            try:
                w = self.i2c.readfrom_mem(a, self.WHO_AM_I, 1)[0]
                if w in (0x68,0x69,0x6A,0x6C):
                    self.addr=a; break
            except OSError: pass
        if self.addr is None: raise OSError("LSM6DS0 not found")
        self.i2c.writeto_mem(self.addr, self.CTRL_REG1_G,  b'\x60')  # ~238 Hz gyro
        self.i2c.writeto_mem(self.addr, self.CTRL_REG6_XL, b'\x60')  # ~238 Hz accel
    def _v(self, reg):
        b = self.i2c.readfrom_mem(self.addr, reg, 6)
        x = ustruct.unpack_from("<h", b, 0)[0]
        y = ustruct.unpack_from("<h", b, 2)[0]
        z = ustruct.unpack_from("<h", b, 4)[0]
        return (x,y,z)
    def accel(self): return self._v(self.OUT_X_L_XL)
    def gyro(self):  return self._v(self.OUT_X_L_G)

i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL), sda=machine.Pin(I2C_SDA), freq=I2C_FREQ)
imu = None
try:
    imu = LSM6DS0(i2c)
    print("[ESP3] IMU OK")
except Exception as e2:
    print("[ESP3] IMU unavailable:", e2, "(use BTN_OFF for testing)")

# Door motion detection (simple spike-on-change metric)
MOTION_WIN_MS      = 200
MOTION_DIFF_THRESH = 3500
MOTION_HOLD_MS     = 120

# ---- Buttons (optional) ----
def mkbtn(pin):
    p = machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP)
    p._prev = 1
    return p
btn_on  = mkbtn(BTN_ON)
btn_off = mkbtn(BTN_OFF)
def fell(p):
    cur = p.value()
    f = (p._prev==1 and cur==0)
    p._prev = cur
    return f

# ---- Baseline mic ----
utime.sleep_ms(300)
acc=0
for _ in range(MIC_BASELINE_SAMPLES): acc += mic_dev(32)
mic_base = acc//MIC_BASELINE_SAMPLES
print("[ESP3] Mic baseline(dev):", mic_base)

# ---- State ----
IDLE=0; RUNNING=1
state = IDLE
hi_since = None
motion_since = None
last_m_t = utime.ticks_ms()
last_vec = None

print("[ESP3] Ready: AUDIO -> 'Motor ON', DOOR MOVE -> 'Motor OFF'.")

while True:
    now = utime.ticks_ms()

    # Audio -> Motor ON
    dev = mic_dev(32)
    if dev > (mic_base + MIC_MARGIN):
        if hi_since is None: hi_since = now
        if state==IDLE and utime.ticks_diff(now, hi_since) >= MIC_HOLD_MS:
            send_str(MSG_ON); state = RUNNING
    else:
        hi_since = None

    # IMU motion -> Motor OFF
    if imu is not None and utime.ticks_diff(now, last_m_t) >= MOTION_WIN_MS:
        try:
            ax,ay,az = imu.accel()
            gx,gy,gz = imu.gyro()
            if last_vec is None:
                last_vec = (ax,ay,az,gx,gy,gz)
            ax0,ay0,az0,gx0,gy0,gz0 = last_vec
            diff = abs(ax-ax0)+abs(ay-ay0)+abs(az-az0)+abs(gx-gx0)+abs(gy-gy0)+abs(gz-gz0)
            last_vec = (ax,ay,az,gx,gy,gz)
            last_m_t = now
            if diff > MOTION_DIFF_THRESH:
                if motion_since is None: motion_since = now
                if state==RUNNING and utime.ticks_diff(now, motion_since) >= MOTION_HOLD_MS:
                    send_str(MSG_OFF); state = IDLE; motion_since = None
            else:
                motion_since = None
        except OSError:
            last_m_t = now

    # Manual overrides
    if fell(btn_on):  send_str(MSG_ON);  state = RUNNING
    if fell(btn_off): send_str(MSG_OFF); state = IDLE

    utime.sleep_ms(10)
