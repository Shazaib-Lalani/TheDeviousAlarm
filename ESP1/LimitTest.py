import network
import espnow
import machine
import time
from machine import Pin
from time import sleep

limitSwitch = Pin(4, Pin.IN, Pin.PULL_UP)
while True:
    print(limitSwitch.value())
    time.sleep(1)