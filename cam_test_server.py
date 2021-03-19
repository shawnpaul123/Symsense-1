import time
import picamera
import picamera.array
from picamera.array import PiRGBArray
import cv2
import numpy as np
import zlib
import base64
import json
import requests
import imutils
import io

ip_addr = 'http://192.168.2.22:5000/predict'
with picamera.PiCamera() as camera:
    camera.resolution = (304,400)
    camera.framerate = 30
    camera.rotation = 270
    camera.start_preview()
    time.sleep(2)
    arr = []
    i = 0
    rawCapture = PiRGBArray(camera, size=(300, 400))
    stream = np.empty((304,400,3),dtype=np.uint8)
    for frame in camera.capture_continuous(rawCapture, format = 'rgb', use_video_port=True):
        image = frame.array
        arr.append(image)
        rawCapture.truncate(0)
        i += 1
        if i == 30:
            break
    '''
    while i < 30:
        camera.capture(stream, format='bgr')
        arr.append(stream)

        with picamera.array.PiRGBArray(camera,size=camera.resolution) as stream:
            camera.capture(stream, format='bgr')
        # At this point the image is available as stream.array
            #print(stream.array.shape)
            image = stream.array
            image = imutils.resize(image,width=400)#print(image)
            arr.append(image)
        '''
    print(arr[0].shape)
    frames = np.array(arr)
    print(frames.shape)
    #frames = np.ndarray(shape=(30,300,400,3),dtype=np.uint8, buffer = arr)
    data = zlib.compress(frames)
    data = base64.b64encode(data)
    data_send = data
    data2 = base64.b64decode(data)
    data2 = zlib.decompress(data2)
    fdata = np.frombuffer(data2, dtype=np.uint8)
    r = requests.post(ip_addr, data={'imgb64' : data_send})
    n = r.json()
    print(type(r))
    result = json.loads(n)#n
    print(result["message"])