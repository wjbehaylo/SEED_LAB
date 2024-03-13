#This file is just where we will be testing out our camera's focal parameters.
#We think a method of determining distance could be using the similar triangles approach, where we assume that
#Moving the same area object further away will have a linear relationship between the size
#Of the image (in pixels) and the distance away that it is at

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
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #we're considering 50 of the 6x6 aruco markers
parameters = aruco.DetectorParameters()
old_quadrant = 4 #this is initialization for old_quadrant, which will help to see when quadrant changes. 4 represents no other state, so change will happen 
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
        Fx = (xWidth * 100)/(14.4)
        Fy = (yWidth * 100)/(14.4)
        #print(f"Focal length x is: {Fx}") #x is 650, y is 635
        #print(f"Focal length y is: {Fy}")


        #This distance is alright, however it only works for looking at it the marker straight on
        DistanceX = (650 * 14.4)/xWidth
        DistanceY = (635 * 14.4)/yWidth
        print(f"Distance calculated with Fx is: {DistanceX}")
        print(f"Distance calculated with Fy is: {DistanceY}")
        break
    cv2.imshow('Frame',frame)
    key = cv2.waitKey(1)
    if (key == ord('q')):
        break

cap.release()
cv2.destroyAllWindows()
    
