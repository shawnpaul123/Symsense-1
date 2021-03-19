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

fig = plt.figure(figsize=( 50 , 50 ))
#!Next Steps:
#https://stackoverflow.com/questions/50331463/convert-rgba-to-rgb-in-python
#make usre if you don't have to 400x400 into face detection, and see if you need to modify face 
#### size before input into lm detection before multile




class variable_holder:

        def __init__(self):
            #read where models are
            self.model_face_loc = './models/face_detector'
            self.model_landmark_loc =  './models/landmark_detector'
            self.model_mask_loc = './models/mask_detector'
            self.prototxtPath = os.path.sep.join([self.model_face_loc, "deploy.prototxt"])
            self.weightsPath = os.path.sep.join([self.model_face_loc,"res10_300x300_ssd_iter_140000.caffemodel"])
            self.confidence = 0.5
            #run landmark code
            self.run_landmarks = False
            self.bs = 32
            self.faceNet = cv2.dnn.readNet(self.prototxtPath, self.weightsPath)
            self.example_mask = ""
            self.example_no_mask = ""
            self.computer_stream = True
            self.landmark_test = './examples/example_01.jpg'
            #load all the models        
            self.maskNet = load_model(self.model_mask_loc)        
            self.landmarkNet = load_model(self.model_landmark_loc)      
            self.faceNet = cv2.dnn.readNet(self.prototxtPath, self.weightsPath)
            #networking vars
            self.ip_server = '192.168.0.22'
            pass
