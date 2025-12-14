import network
import espnow
import machine
from machine import Pin
from time import sleep
import time
import utime

fet = Pin(15, Pin.OUT)

fet.value(0)