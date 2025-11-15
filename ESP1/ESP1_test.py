# esp1.py — Bed node: acts on "Motor ON"/"Motor OFF" strings from ESP3
# MicroPython (ESP32)
import network, espnow, machine, utime

# ====== EDIT THESE ======
MAC_COORD = b"\x14\x2b\x2f\xae\xbe\x44"   # <-- paste ESP3 (door) STA MAC
CHANNEL   = 1
LED_PIN   = 2   # onboard LED on many ESP32 dev boards; change if needed
# ========================

MSG_ON  = "Motor ON"
MSG_OFF = "Motor OFF"

# ---- Radio / ESP-NOW ----
wlan = network.WLAN(network.STA_IF); wlan.active(True)
ap = network.WLAN(network.AP_IF); ap.active(True); ap.config(channel=CHANNEL); ap.active(False)
print("[ESP1] MAC:", wlan.config('mac'))
e = espnow.ESPNow(); e.active(True); e.add_peer(MAC_COORD)

# ---- Motor surrogate ----
led = machine.Pin(LED_PIN, machine.Pin.OUT, value=0)
def motor_on():  led.value(1)
def motor_off(): led.value(0)

print("[ESP1] Ready. LED = motor.")

while True:
    try:
        mac, msg = e.recv(0)
        if mac and msg:
            try:
                s = msg.decode("utf-8")
            except:
                s = str(msg)
            if   s == MSG_ON:
                motor_on();  print("[ESP1] MOTOR ON")
            elif s == MSG_OFF:
                motor_off(); print("[ESP1] MOTOR OFF")
            else:
                print("[ESP1] unhandled:", s)
    except OSError:
        pass
    utime.sleep_ms(5)
