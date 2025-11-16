# esp3.py — Coordinator: "Motor ON"/"Motor OFF" via MIC (quick trigger + auto-release), ESP-NOW
import network, espnow, machine, utime, ustruct

# ====== EDIT THESE ======
MAC_ESP1 = b"\xf4\x65\x0b\x30\x96\xb8"   # ESP1 STA MAC
CHANNEL  = 1
MIC_PIN  = 36                             # ADC1 only (e.g., 36/39/34/35)
I2C_SDA, I2C_SCL, I2C_FREQ = 22, 20, 400000
BTN_ON, BTN_OFF = 33, 32                  # optional test buttons
# ========================

MSG_ON, MSG_OFF, MSG_PING = "Motor ON", "Motor OFF", "PING"

# ---- Radio / ESP-NOW ----
wlan = network.WLAN(network.STA_IF); wlan.active(True)
ap = network.WLAN(network.AP_IF); ap.active(True); ap.config(channel=CHANNEL); ap.active(False)
print("[ESP3] STA MAC:", wlan.config('mac'))
e = espnow.ESPNow(); e.active(True); e.add_peer(MAC_ESP1)

def send_str(s):
    try:
        e.send(MAC_ESP1, s.encode("utf-8"))
        print("[ESP3] sent ->", s)
    except Exception as err:
        print("[ESP3] send err:", err)

# ---- Mic (MAX4466) ----
adc = machine.ADC(machine.Pin(MIC_PIN)); adc.atten(machine.ADC.ATTN_11DB)

def mic_dev(n=16):  # shorter window -> more responsive
    s=0
    for _ in range(n): s += adc.read()
    avg = s//n
    d=0
    for _ in range(n): d += abs(adc.read() - avg)
    return d//n

# Tunables for “sensitive but stable” detection
MIC_WINDOW_SAMPLES = 16
MIC_MARGIN_ON   = 70    # low threshold → triggers easily
MIC_MARGIN_OFF  = 40    # slightly lower to release sooner (hysteresis)
MIC_HOLD_ON_MS  = 200   # brief ON debounce
MIC_HOLD_OFF_MS = 150   # brief OFF debounce
BASELINE_ALPHA  = 0.02  # EMA update when quiet (0.01–0.05 works well)

# ---- IMU optional (unchanged, OK if missing) ----
class LSM6DS0:
    WHO_AM_I, CTRL_REG1_G, CTRL_REG6_XL, OUT_X_L_G, OUT_X_L_XL = 0x0F,0x10,0x20,0x18,0x28
    def __init__(self, i2c):
        self.i2c=i2c; self.addr=None
        for a in (0x6A,0x6B):
            try:
                w=self.i2c.readfrom_mem(a,self.WHO_AM_I,1)[0]
                if w in (0x68,0x69,0x6A,0x6C): self.addr=a; break
            except OSError: pass
        if self.addr is None: raise OSError("LSM6DS0 not found")
        self.i2c.writeto_mem(self.addr,self.CTRL_REG1_G,b'\x60')
        self.i2c.writeto_mem(self.addr,self.CTRL_REG6_XL,b'\x60')
    def _v(self, reg):
        b=self.i2c.readfrom_mem(self.addr,reg,6)
        x=ustruct.unpack_from("<h",b,0)[0]; y=ustruct.unpack_from("<h",b,2)[0]; z=ustruct.unpack_from("<h",b,4)[0]
        return (x,y,z)
    def accel(self): return self._v(self.OUT_X_L_XL)
    def gyro(self):  return self._v(self.OUT_X_L_G)

i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL), sda=machine.Pin(I2C_SDA), freq=I2C_FREQ)
try: imu = LSM6DS0(i2c); print("[ESP3] IMU OK")
except Exception as e: imu=None; print("[ESP3] IMU unavailable:", e)

# ---- Buttons ----
class Button:
    def __init__(self,p): self.pin=machine.Pin(p, machine.Pin.IN, machine.Pin.PULL_UP); self.prev=1
    def fell(self): cur=self.pin.value(); f=(self.prev==1 and cur==0); self.prev=cur; return f
btn_on, btn_off = Button(BTN_ON), Button(BTN_OFF)

# ---- Baseline / State ----
utime.sleep_ms(200)
# quick baseline from a few fast samples
mic_base = sum(mic_dev(MIC_WINDOW_SAMPLES) for _ in range(40))//40
print("[ESP3] Mic baseline(dev):", mic_base)

IDLE, RUNNING = 0, 1
state=IDLE
hi_since=None
lo_since=None
last_ping=utime.ticks_ms()

# IMU motion auto-OFF (kept, but mic also handles OFF now)
MOTION_WIN_MS, MOTION_DIFF_THRESH, MOTION_HOLD_MS = 200, 3500, 120
last_m_t=utime.ticks_ms(); last_vec=None; motion_since=None

print("[ESP3] Ready: short loud sound -> ON; quiet -> auto OFF.")

while True:
    now = utime.ticks_ms()

    # Periodic PING to prove the link
    if utime.ticks_diff(now, last_ping) >= 2000:
        last_ping = now
        send_str(MSG_PING)

    # ---- MIC adaptive baseline + hysteresis ----
    dev = mic_dev(MIC_WINDOW_SAMPLES)

    # Slowly adapt the baseline ONLY when we are near/under the ON threshold
    # (prevents loud periods from “learning in”)
    if dev <= mic_base + MIC_MARGIN_ON:
        mic_base = int((1.0-BASELINE_ALPHA)*mic_base + BASELINE_ALPHA*dev)

    if state == IDLE:
        if dev > mic_base + MIC_MARGIN_ON:
            if hi_since is None: hi_since = now
            if utime.ticks_diff(now, hi_since) >= MIC_HOLD_ON_MS:
                send_str(MSG_ON); state = RUNNING; hi_since=None; lo_since=None
        else:
            hi_since = None
    else:  # RUNNING
        # auto-release when it gets quiet for a short time
        if dev < mic_base + MIC_MARGIN_OFF:
            if lo_since is None: lo_since = now
            if utime.ticks_diff(now, lo_since) >= MIC_HOLD_OFF_MS:
                send_str(MSG_OFF); state = IDLE; lo_since=None; hi_since=None
        else:
            lo_since = None

    # ---- IMU motion also forces OFF (optional) ----
    if imu and utime.ticks_diff(now, last_m_t) >= MOTION_WIN_MS:
        try:
            ax,ay,az=imu.accel(); gx,gy,gz=imu.gyro()
            if last_vec is None: last_vec=(ax,ay,az,gx,gy,gz)
            ax0,ay0,az0,gx0,gy0,gz0=last_vec
            diff=abs(ax-ax0)+abs(ay-ay0)+abs(az-az0)+abs(gx-gx0)+abs(gy-gy0)+abs(gz-gz0)
            last_vec=(ax,ay,az,gx,gy,gz); last_m_t=now
            if diff > MOTION_DIFF_THRESH:
                if motion_since is None: motion_since=now
                if state==RUNNING and utime.ticks_diff(now, motion_since)>=MOTION_HOLD_MS:
                    send_str(MSG_OFF); state=IDLE; motion_since=None
            else:
                motion_since=None
        except OSError:
            last_m_t=now

    # ---- Manual buttons ----
    if btn_on.fell():  send_str(MSG_ON);  state=RUNNING
    if btn_off.fell(): send_str(MSG_OFF); state=IDLE

    utime.sleep_ms(8)
