import sys
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
#import cv2

from gpiozero import LED, Button
import RPi.GPIO as GPIO
import smbus2
import signal

class irSensor:
    '''
    ObjTemperatureReg = 0x07
    AmbTemperatureReg = 0x06
    '''
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


# Camera Code
#camera = PiCamera()
#camera.resolution = (1920, 1080)
#camera.framerate = 30
#rawCapture = PiRGBArray(camera, size=(1920, 1080))
#camera warm-up time
#time.sleep(0.1)
# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
# 	# grab the raw NumPy array representing the image, then initialize the timestamp
# 	# and occupied/unoccupied text
# 	image = frame.array
#   Pipe image into model here
# 	# show the frame
# 	cv2.imshow("Frame", image)
# 	key = cv2.waitKey(1) & 0xFF
# 	# clear the stream in preparation for the next frame
# 	rawCapture.truncate(0)
# 	# if the `q` key was pressed, break from the loop
# 	if key == ord("q"):
# 		break

# IR senosr Test
GPIO.setmode(GPIO.BCM)
irSensor = irSensor(1,0x5a)
#print(irSensor.readAmbTemperature())
print('Object Temperature:' + str(irSensor.readObjTemperature()))
'''
GPIO.setup(11, GPIO.OUT)
GPIO.output(11, GPIO.HIGH)
time.sleep(0.2)
GPIO.output(11, GPIO.LOW)
time.sleep(0.005)
GPIO.output(11, GPIO.HIGH)
time.sleep(0.2)
#camera warm-up time
time.sleep(2)


# Button test to trigger pump

pump = Motor('pump',17,27)
#pump.runMotor()

BUTTON_GPIO = 16
GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def button_pressed_callback(channel):
    pump.runMotor()

GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING,
        callback=button_pressed_callback, bouncetime=100)

signal.signal(signal.SIGINT, signal_handler)
signal.pause()

#Open gate if passing
servo = Motor('servo',22,23)
#servo.runMotor()
'''
#GPIO.cleanup()
