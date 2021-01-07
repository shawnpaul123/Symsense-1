from gpiozero import LED, Button
import RPi.GPIO as GPIO
from smbus2 import SMBus

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout

import sys
import time




class irSensor:
    def __init__(self,port,adress):
        self.bus = smbus.SMBus(port)
        self.address = 0x70

    def write(value):
            bus.write_byte_data(self.address, 0, value)
            return -1

    def readTemperature():
            with SMBus(1) as bus:
            # Read a block of 16 bytes from address 80, offset 0
                temperature = bus.read_i2c_block_data(80, 0, 16)
            # Returned value is a list of 16 bytes
            return int.from_bytes(temperature)

class Motor:
    def __init__(self, motor,pwmPin,directionPin,directionPin2):
        self.motor = motor
        self.speed = pin
        self.forward = directionPin
        self.backward = directionPin2
        self.state = 0

    def runMotor():
        if self.motor == 'pump':
            pwm=GPIO.PWM(self.speed, 100)
            pwm.start(0)
            GPIO.output(self.forward, True)
            GPIO.output(self.backward, False)
            pwm.ChangeDutyCycle(50)
            GPIO.output(self.speed, True)
            sleep(0.5)
            GPIO.output(self.speed, False)
            pwm.stop()

        elif self.motor == 'servo':
            pwm=GPIO.PWM(self.speed, 100)
            if state = 0:
                duty = 90 / 18 + 2
                state = 1
            else:
                duty = 0
                state = 0
            GPIO.output(self.forward, True)
            pwm.ChangeDutyCycle(duty)
            sleep(1)
            GPIO.output(self.forward, False)
            pwm.ChangeDutyCycle(0)
            pwm.stop()


Builder.load_string('''
<CameraClick>:
    orientation: 'vertical'
    Camera:
        id: camera
        resolution: (640, 480)
        play: False
    ToggleButton:
        text: 'Play'
        on_press: camera.play = not camera.play
        size_hint_y: None
        height: '48dp'
    Button:
        text: 'Capture'
        size_hint_y: None
        height: '48dp'
        on_press: root.capture()
''')


class CameraClick(BoxLayout):
    def capture(self):
        '''
        Function to capture the images and give them the names
        according to their captured time and date.
        '''
        camera = self.ids['camera']
        timestr = time.strftime("%Y%m%d_%H%M%S")
        camera.export_to_png("IMG_{}.png".format(timestr))
        print("Captured")


class TestCamera(App):

    def build(self):
        return CameraClick()


TestCamera().run()
