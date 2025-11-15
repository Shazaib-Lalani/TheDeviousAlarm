import time
import esp 
from machine import Pin

#Initialize built in LED pin
led = Pin(13, mode=Pin.OUT)

#blink the LED 10 times
for i in range(10):
	led(1)				#Turn LED on
	time.sleep(1)		#Sleep for 1 second
	led(0)				#Turn LED off
	time.sleep(1)



