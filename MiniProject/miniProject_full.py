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
    
    #faster/prettier messages part 1
    lcd.message= "Desied Location:"

    #this should be constantly running simultaneously as the main body
    while(True):
        if (not Q.empty()):
            lcd.cursor_position(0,1) #faster/prettier part 2. Only change second row

            quadrant = Q.get()
            #I've decided to take this out for now because it wasn't really working
            '''
            if(quadrant == -1):
                output = "No aruco :("            
            else:
            '''
            output = str(quadrant) 
            #print(output) #I may comment this out if it slows down program
            #starttime = time.time()
            lcd.message = output
            #endtime = time.time()
            #print("lcd message time: ", endtime - starttime)
        time.sleep(0.1)



#Arduino Iniialization
ARD_ADDR = 8
i2c_arduino = SMBus(1)




#thread Initialization
Q = queue.Queue()

threadLCD = threading.Thread(target = LCD_Display, args = ())
threadLCD.start()
time.sleep(1) #gives it a second tin initialize


#Camera initialization
cap = cv2.VideoCapture(0) #initializes camera channel
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #we're considering 50 of the 6x6 aruco markers
parameters = aruco.DetectorParameters()
old_quadrant = 4 #this is initialization for old_quadrant, which will help to see when quadrant changes. 4 represents no other state, so change will happen 
time.sleep(0.5)

while True:
        #totalstart = time.time()
        #starttime = time.time()
        ret,frame = cap.read() #get the image from camera. Initially takes long, takes less time after the first capture (startup?)
        #endtime = time.time()
        #print("image capture time: ", endtime - starttime)
        #starttime = time.time()
        #This just draws in lines (in green I think? If we are with BGR?), to make sure that the corner detection algorithm is working.
        #cv2.line(frame,(320,0),(320,480),(0,255,0),1)
        #cv2.line(frame,(0,240),(640,240),(0,255,0),1)
        ##endtime = time.time()
        #print("line drawing time: ", endtime - starttime)
        #starttime = time.time()
        cv2.imshow('Frame',frame) #this should reduce delay a bit maybe?
        #endtime = time.time()
        #print("Imshow time: ", endtime - starttime)
        #displaying image
        key = cv2.waitKey(1) & 0xFF #display the image until q is pressed? The &0xFF is a bitwise mask so that we only consider the byte (where chars are)
        if key == ord('q'): #I think this is just continuous stream until q is pressed. I like
                break
        

        if not ret:
                print("error capturing frame")
                break
        #starttime = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Convert to gray to detect the arucos
        #endtime = time.time()
        #print(f"Grayscale conversion time: ", endtime - starttime)
        

        #corners is a list of numpy arrays. 
        #the first index is which aruco we are considering. So if there are 2 arucos, corners would have 2 numpy arrays in it.
        #the second index represents the corners of the arucos in pixel values. It will have 4 elements (one for each corner). First is top left, other 3 are clockwise from there
        #the third index is a numpy array of the x and y coordinates of the corner (x first)
        #the fourth and final index is selecting the x value or y value
        #so corn
        
        #starttime = time.time()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #this is taking the most time, idk why
        #endtime = time.time()
        #print("Aruco detection time: ", endtime - starttime)


        #If we have detected anything with our aruco.detectMarkers function, we consider which quadrant stuff is in
        if ids is not None: 
            #starttime = time.time()
            xCenterPixel = np.mean(corners[0][0][:, 0]) #Maybe xSum could just be 0000 + 0010 /2 instead of all the x coordinates by 4?
            yCenterPixel = np.mean(corners[0][0][:, 1]) #same with ysum? not too much time complexity though so it shouldn't matter

            #this is a switch statement to calculate which  quadrant the aruco is in. Just returns quadrant, not if it is new or anything.
            #the value of quadrant is what will be put into the LCD display or sent to the arduino
            if xCenterPixel < 320 and yCenterPixel < 240:
                #print("aruco is in the NW Corner") #commenting out testing print statements b/c they slow
                quadrant = 1
            elif xCenterPixel >= 320 and yCenterPixel < 240:
                #print("aruco is in the NE Corner") #commenting out testing print statements b/c they slow
                quadrant = 0
            elif xCenterPixel < 320 and yCenterPixel >= 240:
                #print("aruco is in the SW corner") #commenting out testing print statements b/c they slow
                quadrant = 3
            else:
                #print("aruco is in the SE Corner") #commenting out testing print statements b/c they slow
                quadrant = 2
            quadrant_string = "(" + str(quadrant //2) + ", " + str(quadrant %2) + ")"
            #endtime = time.time()
            #print("Quadrant detection time: ", endtime - starttime)
        else:
            #print("No markers found") #commenting out testing print statements b/c they slow
            quadrant = -1
            quadrant_string = " " * 16

        #needs to be outside detection loop, in case we go from having aruco to not having aruco
        if(quadrant != old_quadrant): #if we have new aruco location:
            #starttime = time.time()
            Q.put(quadrant_string) #this prints it to LCD (or prints that there's no aruco
            #endtime = time.time()
            #print("Q put time: ", endtime - starttime)
            
            #here we will add in arduino send code
            try:
                i2c_arduino.write_byte_data(ARD_ADDR, 1, quadrant) #only sends with new quadrant
            except IOError:
                print("Could not write data to the arduino")
                break
            
            


            old_quadrant = quadrant # state shift
        #otherwise, no need for sending anything anywhere


        
        #break #remove after I'm done with time profiling
        #totalend = time.time()
        #print("A full loop took this many seconds: ", totalend - totalstart)



cap.release()
cv2.destroyAllWindows()


