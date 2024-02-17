import cv2
import cv2.aruco as aruco
import numpy as np
from smbus2 import SMBus
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

cap = cv2.VideoCapture(0)


while True:
        ret,frame = cap.read()

        if not ret:
                print("error capturing frame")
                break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Convert to gray to detect the arucos

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #we're considering 50 of the 6x6 aruco markers
        parameters = aruco.DetectorParameters()

        #corners is a list of numpy arrays. 
        #the first index is which aruco we are considering. So if there are 2 arucos, corners would have 2 numpy arrays in it.
        #the second index represents the corners of the arucos in pixel values. It will have 4 elements (one for each corner). First is top left, other 3 are clockwise from there
        #the third index is a numpy array of the x and y coordinates of the corner (x first)
        #the fourth and final index is selecting the x value or y value
        #so corn
        
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        #these should be the bottom right corners of each quadrant I think? 
        
        nwX = 240 
        nwY = 319
        
        neX = 480
        neY = 319
        
        swX = 240
        swY = 640
        
        seX = 480
        seY = 640

        #If we have detected anything with our aruco.detectMarkers function, we consider which quadrant stuff is in
        if ids is not None: 

            #xSum is the sum of the x coordinates of each corner.
            #ySum is the sum of the y coordinates of each corner. 
            #note that right now, we are only considering for 1 corner (corners[0]). We'd have to do for loop probably to detect which quadrants multiple are in.
            xSum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            ySum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]

            xCenterPixel = xSum*.25 #Maybe xSum could just be 0000 + 0010 /2 instead of all the x coordinates by 4?
            yCenterPixel = ySum*.25 #same with ysum? not too much time complexity though so it shouldn't matter

            #this is a switch statement to calculate which  quadrant the aruco is in. Just returns quadrant, not if it is new or anything.
            #the value of quadrant is what will be put into the LCD display or sent to the arduino
            if xCenterPixel < 240 and yCenterPixel < 320:
                print("aruco is in the NW Corner")
                quadrant = 0
            elif xCenterPixel > 240 and yCenterPixel < 320:
                print("aruco is in the NE Corner")
                quadrant = 1
            elif xCenterPixel < 240 and yCenterPixel > 320:
                print("aruco is in the SW corner")
                quadrant = 3
            else:
                print("aruco is in the SE Corner")
                quadrant = 2
                        
                
        else:
                print("No markers found")
                
        #This just draws in lines (in green I think? If we are with BGR?), to make sure that the corner detection algorithm is working.
        cv2.line(frame,(320,0),(320,480),(0,255,0),3)
        cv2.line(frame,(0,240),(640,240),(0,255,0),3)
        cv2.imshow('Frame',frame)

        key = cv2.waitKey(1) & 0xFF #display the image until q is pressed? The &0xFF is a bitwise mask so that we only consider the byte (where chars are)
        if key == ord('q'):
                break


cap.release()
cv2.destroyAllWindows()
