import os
from tensorflow.keras.models import load_model
import cv2
import PIL.Image
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.video import VideoStream
import numpy as np
import argparse
import imutils
import time
import matplotlib.pyplot as plt
from setting import variable_holder
#!Next Steps:
#https://stackoverflow.com/questions/50331463/convert-rgba-to-rgb-in-python
#make usre if you don't have to 400x400 into face detection, and see if you need to modify face 
#### size before input into lm detection before multile


class socket_setup(variable_holder):
        def __init__(self):
            super().__init__()
            pass

        def setup_server(self):
            pass
            





class detection_ml(variable_holder):

    def __init__(self):
        super().__init__()
        pass

    #detect whether or not face is present
    #inps a resized frame//image of 400
    #outs - return (faces,locs)
    def detect_face(self,frame):
        
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300),(104.0, 177.0, 123.0))
        # pass the blob through the network and obtain the face detections
        #print('blob shape',blob.shape)
        self.faceNet.setInput(blob)
        detections = self.faceNet.forward()
        # initialize our list of faces, their corresponding locations,
        # and the list of predictions from our face mask network
        faces = []
        locs = []
        preds = []
        

        for i in range(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with
            # the detection
            #detections comes from facenet

            conf = detections[0, 0, i, 2]
            # filter out weak detections by ensuring the confidence is
            # greater than the minimum confidence

            if conf > self.confidence:
                # compute the (x, y)-coordinates of the bounding box for
                # the object
                
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                # ensure the bounding boxes fall within the dimensions of
                # the frame
                (startX, startY) = (max(0, startX), max(0, startY))
                (endX, endY) = (min(w - 1, endX), min(h - 1, endY))

                # extract the face ROI, convert it from BGR to RGB channel
                # ordering, resize it to 224x224, and preprocess it
                face = frame[startY:endY, startX:endX]
                face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
                face_landmark = face
                face = cv2.resize(face, (224, 224))               
                face = img_to_array(face)
                face = preprocess_input(face)
                # add the face and bounding boxes to their respective
                # lists              
                faces.append(face)
                locs.append((startX, startY, endX, endY))




        return (faces,locs)


    def detect_mask(self,faces):

        if len(faces) > 0:
        # for faster inference we'll make batch predictions on *all*
        # faces at the same time rather than one-by-one predictions
        # in the above `for` loop
            faces = np.array(faces, dtype="float32")
            preds = self.maskNet.predict(faces, batch_size = self.bs)

        else:
            preds = []


        return preds
            

    #frame is the pic and face are the coordinates
    #!This needs to be reoworked or cancelled
    def detect_landmarks(self,face_list=None):
       
        landmarks = []
        faces = []

        #imread image and make black and white
        #make model predicition
        #return models

        if self.landmark_test:
            #!tensor not able to take the right number

            self.bs = 1 
            rgba_image = PIL.Image.open(self.landmark_test)
            face = rgba_image.convert('RGB') 
            #ensure that you understand and sketch out the approproae functions  
            #face = cv2.imread(self.landmark_test, cv2.IMREAD_GRAYSCALE)            
            face = cv2.resize(np.float32(face), (400, 400))            
            #call detect face location(you already have this info for the else part:)            
            faces,locs = self.detect_face(face)           
            (startX, startY, endX, endY) = locs[0]
            face = face[startY:endY,startX:endX]
            face = cv2.resize(face, (96, 96))  
         
            #make image black and white    
            face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)   
            z = face       
            faces = np.expand_dims(face, axis=0)
            faces = np.expand_dims(faces, axis=-1)
           
        else:

            for face in face_list:
                
                face = cv2.resize(face, (96, 96))  
                #make image black and white    
                face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)         
                face = img_to_array(face)
                face = preprocess_input(face)
                faces.append(face)
       

        return None
        


if __name__ == '__main__':
    pass

    

    