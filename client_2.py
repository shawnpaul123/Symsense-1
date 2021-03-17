import requests
import time
from PIL import Image
import imutils
import numpy as np
import zlib
import base64
from picamera import PiCamera
from picamera.array import PiRGBArray
#import cv2


# server url
URL = "http://127.0.0.1:5000/predict"


def return_image_strings(limit):

    #capture = cv2.VideoCapture(0)
    camera = PiCamera()
    camera.resolution = (1920,1080)
    camera.framerate = 30
    arr = []
    counter_frame = 0
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array()
        image = imutils.resize(image,width=400)
        arr.append(image)
        counter_frame +=1
        if counter_frame == 30:
            break
            
    return arr
    '''
    while counter_frame <=limit:
        isTrue,frame = capture.read()
        frame = imutils.resize(frame, width=400)      
        arr.append(frame)
        counter_frame+=1
    return arr
    '''

if __name__ == "__main__":
    #number of arrays
    s = time.time()
    limit = 30
    frames = return_image_strings(limit)
    frames = np.array(frames) 
    data = zlib.compress(frames)
    data = base64.b64encode(data)
    data_send = data
    data2 = base64.b64decode(data)
    data2 = zlib.decompress(data2)
    fdata = np.frombuffer(data2, dtype=np.uint8)
    r = requests.post("http://127.0.0.1:5000/predict", data={'imgb64' : data_send})
    n = r.json()
    s2 = time.time()
    print(n)
    print('tot time', s2-s)
   
