import network
import espnow
from machine import Pin
from time import sleep
from hcsr04 import HCSR04
from time import sleep

# initialize pin configs 


# initialize esp now
# Initialize WLAN interface for ESP-NOW
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()
# Set WiFi channel to 1 (must match sender)
sta.config(channel=1)
# Initialize ESP-NOW
e = espnow.ESPNow()
e.active(True)

peer = b'\xf4\x65\x0b\x33\x1a\x14'# MAC address of peer's wifi interface 
e.add_peer(peer)

# initialize alarm level
alarm = "0"

while True:
    # load cell read and send
    
    # receive alarm state
    host, msg = e.irecv(1000)
    
    # dictate alarm duties
    if msg
        alarm = msg.decode('utf-8')
        if alarm = "1"
            # motor code
        
        elif alarm = "3"
            # motor and solenoid code
            
        elif alarm = "4"
            #turn off alarm systems
            break
            
            
    
