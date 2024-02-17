#Mini Project:
# Combination of Threaded LCD communication and Aruco Detection

from smbus2 import SMBus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
import cv2
import cv2.aruco as aruco
import numpy as np

#this function will use threading to output onto the LCD based on queue contents
#value is an integer representing the binaray numbers 00 01 10 11 as integers
def LCD_Display():
    
    #initializing the LCD and the LCD connections to the pi
    lcd_columns = 16
    lcd_rows = 2
    i2c_lcd = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)
    lcd.clear()
    lcd.color = [50, 0, 50]

    #this should be constantly running simultaneously as the main body
    while(True):
        if (not Q.empty()):
            quadrant_int = Q.get()
            if(quadrant_int == -1):
                output = "No aruco :("            
            else:
                quadrant_string = str(quadrant_int//2) + " " + str(quadrant_int%2)
                output = f"I got: {quadrant_string}" 
            print(output) #I may comment this out if it slows down program
            lcd.clear()
            lcd.message = output




#I2C address of arduino, set in Arduino Sketch as well
ARD_ADDR = 8
i2c_arduiino = SMBus(1)

Q = queue.Queue()

threadLCD = threading.Thread(target = LCD_Display, args = ())
threadLCD.start()
time.sleep(5) #gives it a second tin initialize

cap = cv2.VideoCapture(0) #initializes camera channel
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #we're considering 50 of the 6x6 aruco markers
parameters = aruco.DetectorParameters()
old_quadrant = 4 #this is initialization for old_quadrant, which will help to see when quadrant changes. 4 represents no other state, so change will happen 
while True:
        ret,frame = cap.read() #get the image from camera

        if not ret:
                print("error capturing frame")
                break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Convert to gray to detect the arucos

        

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
                #print("aruco is in the NW Corner") #commenting out testing print statements b/c they slow
                quadrant = 1
            elif xCenterPixel > 240 and yCenterPixel < 320:
                #print("aruco is in the NE Corner") #commenting out testing print statements b/c they slow
                quadrant = 0
            elif xCenterPixel < 240 and yCenterPixel > 320:
                #print("aruco is in the SW corner") #commenting out testing print statements b/c they slow
                quadrant = 3
            else:
                #print("aruco is in the SE Corner") #commenting out testing print statements b/c they slow
                quadrant = 2
                        
        
        else:
            #print("No markers found") #commenting out testing print statements b/c they slow
            quadrant = -1

        #needs to be outside detection loop, in case we go from having aruco to not having aruco
        if(quadrant != old_quadrant): #if we have new aruco location
            Q.put(quadrant) #this prints it to LCD (or prints that there's no aruco
            
            #here we will add in arduino send code



            old_quadrant = quadrant # state shift
        #otherwise, no need for sending anything anywhere
                
        #This just draws in lines (in green I think? If we are with BGR?), to make sure that the corner detection algorithm is working.
        cv2.line(frame,(320,0),(320,480),(0,255,0),3)
        cv2.line(frame,(0,240),(640,240),(0,255,0),3)
        cv2.imshow('Frame',frame)

        key = cv2.waitKey(1) & 0xFF #display the image until q is pressed? The &0xFF is a bitwise mask so that we only consider the byte (where chars are)
        if key == ord('q'): #I think this is just continuous stream until q is pressed. I like
                break


cap.release()
cv2.destroyAllWindows()


