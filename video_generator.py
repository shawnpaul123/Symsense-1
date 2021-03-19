
'''
credits: https://github.com/Danotsonof/facial-landmark-detection/blob/master/facial-landmark.ipynb
where models were got from
# save face detection algorithm's url in haarcascade_url variable
haarcascade_url = "https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_alt2.xml"
# save facial landmark detection model's url in LBFmodel_url variable
LBFmodel_url = "https://github.com/kurnianggoro/GSOC2017/raw/master/data/lbfmodel.yaml"
'''
import cv2
import os
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.video import VideoStream
import numpy as np
import argparse
import imutils
import time
from setting import variable_holder
from model_script import detection_ml



class image_processing(variable_holder):
    #import all variables from setting class
    def __init__(self):
        super().__init__()

    #placeholder
    def plot_image(self):
        pass

    #ignore anything with 0
    #useful for sorting confidence arrays
    def filter_func(self,val):
        if val == 0:
            return False
        else:
            return True

    #runs video capture and display script on local computer
    def mvp_video_stream(self):
        # initialize the video stream and allow the camera sensor to warm up
        print("[INFO] starting video stream...")
        #declare detection_ml model
        dml = detection_ml()
        #initialize arry to take average values
        arr_mask,arr_withoutMask = [0]*10,[0]*10
        #indexer for avg array
        j=0    
     

        if self.computer_stream:
            vs = VideoStream(src=0).start()
        # loop over the frames from the video stream        
     
        while True:
            
            # grab the frame from the threaded video stream and resize it
            # to have a maximum width of 400 pixels
            #########################################################################################
            if self.computer_stream:
                frame = vs.read()
                frame = imutils.resize(frame, width=400)
            # detect faces in the frame and determine if they are wearing a
            # face mask or not

            else:
                pass
                #this is where you have code to get video stream from rpi


            #########################################################################################


            #!This is where you would have to send/receive a request/response
            
            faces,locs = dml.detect_face(frame)
            preds = dml.detect_mask(faces)

            if len(preds) == 0:
                continue

            # loop over the detected face locations and their corresponding
            # locations
            for (box, pred,face) in zip(locs, preds,faces):
                # unpack the bounding box and predictions
                (startX, startY, endX, endY) = box
                #get back the landmarks for each face
                if j == 9:
                    arr_mask,arr_withoutMask = [0]*10,[0]*10
                    j=0               
                arr_mask[j],arr_withoutMask[j] = pred  
                #get adjusted values   
                adj_mask = list(filter(self.filter_func,arr_mask))
                adj_withoutMask = list(filter(self.filter_func,arr_withoutMask))
                #print('adj_vales',adj_mask,adj_withoutMask)
                               
                if len(adj_mask) == 1:
                    adj_mask = adj_mask[0]
                else:
                    adj_mask = sum(adj_mask)/len(adj_withoutMask)               
                if len(adj_withoutMask) == 1:
                    adj_withoutMask = adj_withoutMask[0]
                else:
                    adj_withoutMask = sum(adj_withoutMask)/len(adj_withoutMask)
                    
                j+=1 
                
                # determine the class label and color we'll use to draw
                # the bounding box and text
                label = "Mask" if adj_mask > adj_withoutMask else "No Mask"
                color = (0, 255, 0) if label == "Mask" else (0, 0, 255)
                # include the probability in the label
                label = "{}: {:.2f}%".format(label, max(adj_mask, adj_withoutMask) * 100)
                # display the label and bounding box rectangle on the output
                # frame
                cv2.putText(frame, label, (startX, startY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
                cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)
                #add value to index 
                


            # show the output frame
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) 
        # do a bit of cleanup
        cv2.destroyAllWindows()
        vs.stop()



#python -m pip install --user opencv-contrib-python

if __name__ == '__main__':
    #https://www.pyimagesearch.com/2018/09/26/install-opencv-4-on-your-raspberry-pi/
    imp = image_processing()
    imp.mvp_video_stream()
    
    



