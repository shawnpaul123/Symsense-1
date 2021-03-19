import RPi.GPIO as GPIO
import time

# initial setup
servoPin = 22
buttonPin = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPin, GPIO.OUT)
GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

p = GPIO.PWM(servoPin, 50) # 50Hz
p.start(0) # Initialization
motorPos = 1

while True: # Run forever
    if GPIO.input(buttonPin) == GPIO.HIGH:
        p.ChangeDutyCycle(motorPos)
        motorPos = motorPos + 1
    # slight pause to debounce
    time.sleep(0.2)
    
# should add this to stop motor at the end
p.stop()
GPIO.cleanup()