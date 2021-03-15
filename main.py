import sys
sys.path.insert(1, "./lib")
import os
import logging
import traceback
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import requests
from PIL import Image
import imutils
import numpy as np
import zlib
import base64
import json


from gpiozero import LED, Button
import RPi.GPIO as GPIO
import smbus2
import signal
from waveshare_epd import epd2in9d
from PIL import Image,ImageDraw,ImageFont

class irSensor:
    def __init__(self,port,address):
        self.bus = smbus2.SMBus(port)
        self.address = address
        self.ObjTemperatureReg = 0x07
        self.AmbTemperatureReg = 0x06

    def readObjTemperature(self):
            data = self.bus.read_word_data(self.address, self.ObjTemperatureReg)
            # Input voltage compensation, convert to K, convert to C
            temperature = ((data-(.3*0.6)) * 0.02) - 273.15
            return temperature

    def readAmbTemperature(self):
            data = self.bus.read_word_data(self.address, self.AmbTemperatureReg)
            # Input voltage compensation, convert to K, convert to C
            temperature = ((data-(.3*0.6)) * 0.02) - 273.15
            return temperature

class Motor:
    def __init__(self, motor,*argv):
        self.motor = motor
        try:
            self.enablePin = argv[0]
            GPIO.setup(self.enablePin,GPIO.OUT)
            self.inputPin1 = argv[1]
            GPIO.setup(self.inputPin1,GPIO.OUT)
            self.inputPin2 = argv[2]
            GPIO.setup(self.inputPin2,GPIO.OUT)
        except IndexError:
            pass

        self.state = 0

    def runMotor(self):
        if self.motor == 'pump':
            # Input Pin 2 wired to ground, turn on for 0.5 seconds to dispense
            GPIO.output(self.enablePin, True)
            GPIO.output(self.inputPin1, True)
            time.sleep(2)
            GPIO.output(self.enablePin, False)
            GPIO.output(self.inputPin1, False)

        elif self.motor == 'servo':
            # Probably can also wire input Pin 2 to ground since we don't want to switch polarity?
            # Pulse Enable Pin to servo, keeping track of polarity for direction
            # Frequency 250 (4ms period)
            p = GPIO.PWM(self.enablePin, 50) # 50Hz
            p.start(0) # Initialization
            if self.state == 0:
                # might need to test duty cycle and see how the motor rotates between 0 and 90 deg
                # 2ms Pulse for +90deg or 1ms pulse for -90 deg
                duty = 100
                self.state = 1

            else:
                #1.5 ms pulse for 0 deg
                duty = 50
                self.state = 0
            p.ChangeDutyCycle(duty)

class Camera:
    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (1920,1080)
        self.camera.framerate = 30
    def maskCheck(self):
        rawCapture = PiRGBArray(self.camera, size=(1920, 1080))
        arr = []
        i = 0
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array()
            image = imutils.resize(image,width=400)
            arr.append(image)
            i+=1
            if i == 30:
                break
        frames = np.array(arr)
        data = zlib.compress(frames)
        data = base64.b64encode(data)
        data_send = data
        data2 = base64.b64decode(data)
        data2 = zlib.decompress(data2)
        fdata = np.frombuffer(data2, dtype=np.uint8)
        r = requests.post("http://127.0.0.1:5000/predict", data={'imgb64' : data_send})
        n = r.json()
        result = json.loads(n)
        return str(result["message"])

def resetScreen():
    global epd, draw, image, font24
    epd.Clear(0xFF)
    draw.text((0, 0), '', font = font24, fill = 0)
    draw.text((0, 20), '', font = font24, fill = 0)
    draw.text((0, 40), '', font = font24, fill = 0)
    draw.text((0, 60), '', font = font24, fill = 0)
    draw.text((0, 80), '', font = font24, fill = 0)
    draw.text((0, 100), '', font = font24, fill = 0)
    # draw image and hold for 5 sec
    epd.display(epd.getbuffer(image))

def button_pressed_callback(channel):
    global draw, font24, epd, image, pump, irSensor, servo, camera
    resetScreen()
    draw.text((0, 40), 'Dispensing Hand Sanitizer', font = font24, fill = 0)
    epd.display(epd.getbuffer(image))
    pump.runMotor()
    resetScreen()
    draw.text((0, 60), 'Please face camera for Mask Check', font = font24, fill = 0)
    epd.display(epd.getbuffer(image))
    i = 0
    mask = ''
    while(mask != 'mask'):
        mask = camera.maskCheck()
        if mask == 'not mask':
            draw.text((0, 60), 'Please put mask on properly', font = font24, fill = 0)
            draw.text((0, 80), 'and face camera', font = font24, fill = 0)
            epd.display(epd.getbuffer(image))
        if i == 2:
            break
        i+=1
    resetScreen()
    if mask == 'mask':
        draw.text((0, 80), 'Please place forehead near', font = font24, fill = 0)
        draw.text((0, 100), 'temperature sensor', font = font24, fill = 0)
        epd.display(epd.getbuffer(image))
        temp = irSensor.readObjTemperature()
        resetScreen()
        if temp < 38:
            servo.runMotor()
            draw.text((0, 100), 'Pass', font = font24, fill = 0)
            epd.display(epd.getbuffer(image))
            time.sleep(5)
            servo.runMotor()
        else:
            draw.text((0, 100), 'Fail', font = font24, fill = 0)
            epd.display(epd.getbuffer(image))
            time.sleep(5)
    else:
        draw.text((0, 100), 'Fail', font = font24, fill = 0)
        epd.display(epd.getbuffer(image))
        time.sleep(5)

    epd.Clear(0xFF)
    draw.text((0, 0), 'SymSense', font = font24, fill = 0)
    draw.text((0, 20), 'Push Button to Begin', font = font24, fill = 0)
    draw.text((0, 40), '', font = font24, fill = 0)
    draw.text((0, 60), '', font = font24, fill = 0)
    draw.text((0, 80), '', font = font24, fill = 0)
    draw.text((0, 100), '', font = font24, fill = 0)
    # draw image and hold for 5 sec
    epd.display(epd.getbuffer(image))

def setup():
    global pump, irSensor, servo, camera, draw, font24, epd, image
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    BUTTON_GPIO = 27
    SERVO_GPIO = 22
    PUMP_ENABLE = 13
    PUMP_DIRECTION1 = 5
    PUMP_DIRECTION2 = 6
    pump = Motor('pump',PUMP_ENABLE,PUMP_DIRECTION1,PUMP_DIRECTION2)
    irSensor = irSensor(1,0x5a)
    servo = Motor('servo',SERVO_GPIO)
    camera = Camera()
    GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(BUTTON_GPIO, GPIO.RISING,
            callback=button_pressed_callback, bouncetime=300)


    # Screen
    picdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pic')
    libdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib')

    # set fonts
    font24 = ImageFont.truetype(os.path.join(picdir, 'Font.ttc'), 24)
    font16 = ImageFont.truetype(os.path.join(picdir, 'Font.ttc'), 16)

    # init screen + clear
    epd = epd2in9d.EPD()
    epd.init()
    epd.Clear(0xFF)

    #  start drawing image
    image = Image.new('1', (epd.height, epd.width), 255)  # 255: clear the frame
    draw = ImageDraw.Draw(image)
    draw.text((0, 0), 'SymSense', font = font24, fill = 0)
    draw.text((0, 20), 'Push Button to Begin', font = font24, fill = 0)
    draw.text((0, 40), '', font = font24, fill = 0)
    draw.text((0, 60), '', font = font24, fill = 0)
    draw.text((0, 80), '', font = font24, fill = 0)
    draw.text((0, 100), '', font = font24, fill = 0)

    # draw image and hold for 5 sec
    epd.display(epd.getbuffer(image))

def main():
    print('SymSense Start')
    while True:
        time.sleep(1)

if __name__ == "__main__":
    setup()
    main()
    GPIO.cleanup()
    epd.Clear(0xFF)
    epd.sleep()
    time.sleep(1)
    epd.Dev_exit()
