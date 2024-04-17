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
        angle = (xCenter - 640)/640 *31
        return angle



def LCD_Display():
    
    #initializing the LCD and the LCD connections to the pi
    lcd_columns = 16
    lcd_rows = 2
    i2c_lcd = board.I2C()

    lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)
    lcd.clear()
    lcd.color = [50, 0, 50]
    
    #faster/prettier messages part 1
    lcd.message= "Angle Detected:"

    #this should be constantly running simultaneously as the main body
    while(True):
        if (not Q.empty()):
            lcd.cursor_position(0,1) #faster/prettier part 2. Only change second row
            
            angle = Q.get()
            #I've decided to take this out for now because it wasn't really working
            '''
            if(quadrant == -1):
                output = "No aruco :("            
            else:
            '''
            output = str(angle) 
            #print(output) #I may comment this out if it slows down program
            #starttime = time.time()
            lcd.message = output
            #endtime = time.time()
            #print("lcd message time: ", endtime - starttime)

#Arduino Iniialization
ARD_ADDR = 8
i2c_arduino = SMBus(1)


#thread Initialization
Q = queue.Queue()

threadLCD = threading.Thread(target = LCD_Display, args = ())
threadLCD.start()
time.sleep(1) #gives it a second tin initialize


cap = cv2.VideoCapture(0) #initializes camera channel
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #we're considering 50 of the 6x6 aruco markers
parameters = aruco.DetectorParameters()
old_quadrant = 4 #this is initialization for old_quadrant, which will help to see when quadrant changes. 4 represents no other state, so change will happen 
time.sleep(0.5)
last_angle = 40
while True:
    time.sleep(0.1) #this is so that we don't over-capture aruco angle
    ret,frame = cap.read()
    if not ret:
            print("error capturing frame")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #this is taking the most time, idk why

    #We only care about the x pixel of the center for now, but I am recording y just in case
    if ids is not None:
        

        #xWidth is the top right x - top left x + bottom right x - bottom left x.
        #I decided to add in the top and bottom and take the average in case there is any tangential distortion
        xWidth = (corners[0][0][1][0] - corners[0][0][0][0] + corners[0][0][2][0] - corners[0][0][3][0])/2

        #yWidth should be bottom left y minus top left y + bottom right y 0 top right y /2
        yWidth = (corners[0][0][3][1] - corners[0][0][0][1] + corners[0][0][2][1] - corners[0][0][1][1])/2

        
        #Here we are using the equation F = (P x D)/W
        #P is going to be the width, D is the distance we place it at initially, (1 meter or 100 cm), W is the size (14.4 cm)
        Fx = (xWidth * 100)/(5)
        Fy = (yWidth * 100)/(5)
        print(f"Focal length x is: {Fx}") #x is 650, y is 635
        print(f"Focal length y is: {Fy}")

        #x center is top right + bottom right /2 (to get the average right most coord) - xWidth/2 to get
        xCenter = int((corners[0][0][1][0] + corners[0][0][2][0])/2 - xWidth/2)
        #y center is bottom left + bottom_right/2 - yWidth/2
        yCenter = int((corners[0][0][3][1] + corners[0][0][2][1])/2 - yWidth/2)

        #based on our xCenter, we get The aruco angle
        angle = Get_Angle(xCenter)
        #if(angle > last_angle+0.5 or angle < last_angle-0.5):
        Q.put("{:.3f}".format(angle))
        last_angle = angle
        #this should draw a cross over the center of the aruco
        cv2.line(frame, (xCenter,yCenter),(xCenter,yCenter), (0,255,0),5)
        #cv2.line(frame, (xCenter-1, yCenter),(xCenter + 1,yCenter), (0,255,0),1)

        
        #This distance is alright, however it only works for looking at it the marker straight on
        DistanceX = (650 * 14.4)/xWidth
        DistanceY = (635 * 14.4)/yWidth
        print(f"Distance calculated with Fx is: {DistanceX}")
        print(f"Distance calculated with Fy is: {DistanceY}")
        #break
        
    cv2.imshow('Frame',frame)
    key = cv2.waitKey(1)
    if (key == ord('q')):
        break

    ARD_ADDR = 8
    """
    try:
        #Do we need an offset?
        i2c_arduino.write_i2c_block_data(ARD_ADDR, offset, angle)
    except IOError:
        print("Could not write data to the arduino")
    """

cap.release()
cv2.destroyAllWindows()
    
