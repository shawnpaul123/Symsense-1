import RPi.GPIO as GPIO
import time

buttonPin = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

while True: # Run forever
    if GPIO.input(buttonPin) == GPIO.HIGH:
        print("Button was pushed!")
    #slight pause to debounce
    time.sleep(0.2)