from tkinter import *
from PIL import ImageTk, Image
import time
import threading
import sys

from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2

from gpiozero import LED, Button
import RPi.GPIO as GPIO
import smbus2
import signal


class irSensor:
    ObjTemperatureReg = 0x07
    AmbTemperatureReg = 0x06
    def __init__(self,port,address):
        self.bus = smbus2.SMBus(port)
        self.address = address

    def readObjTemperature(self):
            data = self.bus.read_word_data(self.address, ObjTemperatureReg)
            # Input voltage compensation, convert to K, convert to C
            temperature = ((data-(.3*0.6)) * 0.02) - 273.15
            return temperature

    def readAmbTemperature(self):
            data = self.bus.read_word_data(self.address, AmbTemperatureReg)
            # Input voltage compensation, convert to K, convert to C
            temperature = ((data-(.3*0.6)) * 0.02) - 273.15
            return temperature

class Motor:
    def __init__(self, motor,*argv):
        self.motor = motor
        try:
            self.enablePin = arg[0]
            GPIO.setup(self.enablePin,GPIO.OUT)
            self.inputPin1 = arg[1]
            GPIO.setup(self.inputPin1,GPIO.OUT)
            self.inputPin2 = arg[2]
            GPIO.setup(self.inputPin2,GPIO.OUT)
        except IndexError:
            pass

        self.state = 0

    def runMotor(self):
        if self.motor == 'pump':
            # Input Pin 2 wired to ground, turn on for 0.5 seconds to dispense
            GPIO.output(self.enablePin, True)
            GPIO.output(self.inputPin1, True)
            time.sleep(0.5)
            GPIO.output(self.enablePin, False)
            GPIO.output(self.inputPin1, False)

        elif self.motor == 'servo':
            # Probably can also wire input Pin 2 to ground since we don't want to switch polarity?
            # Pulse Enable Pin to servo, keeping track of polarity for direction
            # Frequency 250 (4ms period)
            pwm=GPIO.PWM(self.enablePin, 250)
            GPIO.output(self.inputPin1, True)

            if self.state == 0:
                # might need to test duty cycle and see how the motor rotates between 0 and 90 deg
                # 2ms Pulse for +90deg or 1ms pulse for -90 deg
                duty = 50
                self.state = 1

            else:
                #1.5 ms pulse for 0 deg
                duty = 37.5
                self.state = 0

            pwm.start(duty)
            time.sleep(1)
            pwm.ChangeDutyCycle(0)
            pwm.stop()
            GPIO.output(self.inputPin1, False)


class Camera:
    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (1920,1080)
        self.camera.framerate = 30
    def maskCheck():
        return True

def CovidScreen():
    global screen, pump, camera, irSensor, gate
    screen.grid_forget()
    screen = Label(image=image_list[1])
    screen.grid(row=5,column=0,columnspan=3)
    pump.runMotor()

    screen.grid_forget()
    screen = Label(image=image_list[2])
    screen.grid(row=5,column=0,columnspan=3)
    maskDetected = camera.maskCheck()

    screen.grid_forget()
    screen = Label(image=image_list[3])
    screen.grid(row=5,column=0,columnspan=3)
    temperature = irSensor.readObjTemperature()
    testPassed = False
    if temperature < 38 and maskDetected:
        testPassed = True
        screen.grid_forget()
        screen = Label(image=image_list[4])
        screen.grid(row=5,column=0,columnspan=3)
        gate.open()

    else:
        screen.grid_forget()
        screen = Label(image=image_list[5])
        screen.grid(row=5,column=0,columnspan=3)
        # Wait for a bit before reset
    event = threading.Event()
    event.wait(5)
    if testPassed:
        gate.close()
    #reset screen
    screen.grid_forget()
    screen = Label(image=image_list[0])
    screen.grid(row=5,column=0,columnspan=3)


def button_callback(channel):
    global screen
    screen.grid_forget()
    screen = Label(image=image_list[1])
    screen.grid(row=5,column=0,columnspan=3)
    CovidScreen()

root = Tk()
root.title('SymSense')
#for testing
b1=Button(root,text="Read Temperature Sensor",command=lambda:temperatureSensor())
temperature = 38
l1 = Label(root,text=str(temperature)+'C')
b2=Button(root,text="Start Camera",command=lambda:camera())
b3=Button(root,text="Dispense Hand Sanitizer",command=lambda:dispenseHS())
b4=Button(root,text="Open Gate",command=lambda:openGate())
button_quit = Button(root,text='Exit Program',command=root.quit)
b1.grid(row=0,column=0)
l1.grid(row=0,column=1)
b2.grid(row=1,column=0)
b3.grid(row=2,column=0)
b4.grid(row=3,column=0)
button_quit.grid(row=4,column=0)

zoom = 1.5
#multiple image size by zoom
image = Image.open('img/picamera.png')
pixels_x, pixels_y = tuple([int(zoom * x)  for x in image.size])
my_img = ImageTk.PhotoImage(Image.open('img/picamera.png').resize((pixels_x, pixels_y)))
my_img1 = ImageTk.PhotoImage(Image.open('img/camera.PNG').resize((pixels_x, pixels_y)))
my_img2 = ImageTk.PhotoImage(Image.open('img/camera_success.PNG').resize((pixels_x, pixels_y)))
my_img3 = ImageTk.PhotoImage(Image.open('img/camera_fail.PNG').resize((pixels_x, pixels_y)))
my_img4 = ImageTk.PhotoImage(Image.open('img/temperature.PNG').resize((pixels_x, pixels_y)))
my_img5 = ImageTk.PhotoImage(Image.open('img/fail.PNG').resize((pixels_x, pixels_y)))
image_list = [my_img, my_img1, my_img2, my_img3, my_img4, my_img5]

screen = Label(image=my_img)
screen.grid(row=5,column=0,columnspan=2,rowspan=2)

GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(16, GPIO.FALLING, callback=button_callback, bouncetime=300)

irSensor = irSensor(1,0x5a)
pump = Motor('pump',17,27)
camera = Camera()


while True:
    root.update_idletasks()
    root.update()

GPIO.cleanup()
