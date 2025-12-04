import network
import espnow
import machine
from machine import Pin
from time import sleep
import time
import utime

fet = Pin(15, Pin.OUT)

while True:
    fet.value(0)
    time.sleep(1)
    fet.value(1)
    time.sleep(1)