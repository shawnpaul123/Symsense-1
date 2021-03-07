from picamera import PiCamera
from time import sleep

cam = PiCamera()

cam.start_preview()
sleep(5)
cam.capture('/home/pi/Documents/Symsense/cam_pics/test_image.jpg')
cam.stop_preview()