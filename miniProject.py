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

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters()

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        nwX = 240
        nwY = 319
        
        neX = 480
        neY = 319
        
        swX = 240
        swY = 640
        
        seX = 480
        seY = 640

        if ids is not None:
            xSum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            ySum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]

            xCenterPixel = xSum*.25
            yCenterPixel = ySum*.25
            
            if xCenterPixel < 240 and yCenterPixel < 320 and quadrant != 0:
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
                
        
        cv2.line(frame,(320,0),(320,480),(0,255,0),3)
        cv2.line(frame,(0,240),(640,240),(0,255,0),3)
        cv2.imshow('Frame',frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
                break


cap.release()
cv2.destroyAllWindows()
