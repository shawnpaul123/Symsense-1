import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(5,GPIO.OUT)
GPIO.setup(6,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)

GPIO.output(13, GPIO.HIGH)
GPIO.output(5, GPIO.LOW)
GPIO.output(6, GPIO.HIGH)
input()
GPIO.output(13, GPIO.LOW)
GPIO.cleanup()