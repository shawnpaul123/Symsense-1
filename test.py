from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout

import sys
import time
from picamera import PiCamera

from gpiozero import LED, Button
import RPi.GPIO as GPIO
import smbus2
from mlx90614 import MLX90614

class irSensor:
    def __init__(self,port,address):
        self.bus = smbus2.SMBus(port)
        self.address = address

    def readObjTemperature(self):
            data = self.bus.read_word_data(self.address, 0x07)
            temperature = ((data-(.3*0.6)) * 0.02) - 273.15
            return temperature
        
    def readAmbTemperature(self):
            data = self.bus.read_word_data(self.address, 0x06)
            temperature = ((data-(.3*0.6)) * 0.02) - 273.15
            return temperature

class Motor:
    def __init__(self, motor,enablePin,inputPin):
        self.motor = motor
        self.enablePin = enablePin
        self.inputPin = inputPin
        self.state = 0
        GPIO.setup(self.enablePin,GPIO.OUT)
        GPIO.setup(self.inputPin,GPIO.OUT)

    def runMotor(self):
        if self.motor == 'pump':
            GPIO.output(self.enablePin, True)
            GPIO.output(self.inputPin, True)            
            time.sleep(0.5)
            GPIO.output(self.enablePin, False)
            GPIO.output(self.inputPin, False)

        elif self.motor == 'servo':
            pwm=GPIO.PWM(self.speed, 100)
            if self.state == 0:
                duty = 90 / 18 + 2
                self.state = 1
            else:
                duty = 0
                self.state = 0
            GPIO.output(self.forward, True)
            pwm.ChangeDutyCycle(duty)
            time.sleep(1)
            GPIO.output(self.forward, False)
            pwm.ChangeDutyCycle(0)
            pwm.stop()




#camera = PiCamera()
#camera.resolution = (3280, 2464)
#camera.start_preview()

GPIO.setmode(GPIO.BCM)
motor = Motor('pump',17,27)
#motor.runMotor()                                                                                                                

irSensor = irSensor(1,0x5a)
print(irSensor.readAmbTemperature())
print(irSensor.readObjTemperature())
GPIO.setup(11, GPIO.OUT)
GPIO.output(11, GPIO.HIGH)
time.sleep(0.2)
GPIO.output(11, GPIO.LOW)
time.sleep(0.005)
GPIO.output(11, GPIO.HIGH)
time.sleep(0.2)
#camera warm-up time
time.sleep(2)
GPIO.cleanup()
#camera.capture("image.jpg")
