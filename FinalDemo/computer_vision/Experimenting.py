from smbus2 import SMBus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
import cv2
import cv2.aruco as aruco
import numpy as np


#this function will take in the xCenter we detect and output the angle that it is at
def Get_Angle(xCenter):
        #angle = (xCenter - 320)/320 * 30 #this is assuming that our program has 30 degrees on both sides
        angle = (xCenter - 640)/640 *30.7
        return angle



#This file is just where we will be testing out some distance calculation things. 
#We determined the focal length from Focal_Length_Detector.py
#and now we are going to try to use trig to enable it to detect aruco markers at angles

from smbus2 import SMBus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
import cv2
import cv2.aruco as aruco
import numpy as np


cap = cv2.VideoCapture(0) #initializes camera channel
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_EXPOSURE, -14)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #we're considering 50 of the 6x6 aruco markers
parameters = aruco.DetectorParameters()
time.sleep(0.5)

while True:

    ret,frame = cap.read()
    if not ret:
            print("error capturing frame")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #this is taking the most time, idk why

    if ids is not None:
        #xWidth is the top right x - top left x + bottom right x - bottom left x.
        #I decided to add in the top and bottom and take the average in case there is any tangential distortion
        xWidth = (corners[0][0][1][0] - corners[0][0][0][0] + corners[0][0][2][0] - corners[0][0][3][0])/2

        #yWidth should be bottom left y minus top left y + bottom right y 0 top right y /2
        yWidth = (corners[0][0][3][1] - corners[0][0][0][1] + corners[0][0][2][1] - corners[0][0][1][1])/2

        #Here we are using the equation F = (P x D)/W
        #P is going to be the width, D is the distance we place it at initially, (1 meter or 100 cm), W is the size (14.4 cm)
        #Fx = (xWidth * 100)/(5)
        #Fy = (yWidth * 100)/(5)
        #print(f"Focal length x is: {Fx}") #x is 1110 and y is 1110 for a 5 cm by 5 cm aruco
        #print(f"Focal length y is: {Fy}")
        

        #This distance is alright, however it only works for looking at it the marker straight on
        DistanceX = (1110 * 5)/xWidth
        DistanceY = (1110 * 5)/yWidth 
        print(f"Distance calculated with Fx is: {DistanceX}")
        print(f"Distance calculated with Fy is: {DistanceY}")
        
        #x center is top right + bottom right /2 (to get the average right most coord) - xWidth/2 to get the middle of that
        xCenter = int((corners[0][0][1][0] + corners[0][0][2][0])/2 - xWidth/2)
        #y center is bottom left + bottom_right/2 - yWidth/2
        yCenter = int((corners[0][0][3][1] + corners[0][0][2][1])/2 - yWidth/2)
        #based on our xCenter, we get The aruco angle
        angle = Get_Angle(xCenter)
        print(f"Angle calculated is: {angle}")
        time.sleep(1) #so after we detect one and output angle, we wait a second before repeating
        
    cv2.imshow('Frame',frame)
    key = cv2.waitKey(1)
    if (key == ord('q')):
        break