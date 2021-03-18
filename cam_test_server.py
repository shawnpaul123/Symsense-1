import time
import picamera
import picamera.array
import cv2

with picamera.PiCamera() as camera:
    camera.resolution = (300,400)
    camera.framerate = 30
    camera.start_preview()
    time.sleep(2)
    arr = []
    while i < 30:
        with picamera.array.PiRGBArray(camera) as stream:
            camera.capture(stream, format='bgr')
        # At this point the image is available as stream.array
            image = stream.array
            #print(image)
            arr.append(image)
            i +=1
    frames = np.array(arr)
        #print(frames.shape)
    data = zlib.compress(frames)
    data = base64.b64encode(data)
    data_send = data
    data2 = base64.b64decode(data)
    data2 = zlib.decompress(data2)
    fdata = np.frombuffer(data2, dtype=np.uint8)
    r = requests.post(self.ip_addr, data={'imgb64' : data_send})
    n = r.json()
    print(type(r))
    result = json.loads(n)#n
    print(result["message"])