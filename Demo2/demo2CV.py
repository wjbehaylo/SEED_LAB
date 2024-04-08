#EENG 350: SEED LAB
#FSM for Demo 2 CV

from smbus2 import SMBus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
import cv2
import cv2.aruco as aruco
import numpy as np

def Get_Angle(xCenter):
    angle = (xCenter -640)/640 * 30.7 #this 30.7 will have to change
    return angle

#Not sure if this is necessary for this portion
#I am going to comment it out for simplicity
'''
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
            
            
            output = str(angle) 
            #print(output) #I may comment this out if it slows down program
            #starttime = time.time()
            lcd.message = output
            #endtime = time.time()
            #print("lcd message time: ", endtime - starttime)
'''

#honestly I (walter) just made the state machine because I think it will be important for the final demo. 
# Here, the only state that will be gone through again is "searching"

#States:
# A: initialize
#       where the program starts, starting camera, establishing I2C connection with Arduino
# B: searching for marker
#       This state is when the robot has started rotating
#       It searches/rotates until the marker is found, then exiting the state
# C: marker found
#       Once the marker is found, we need to do the math to figure out what angle it is at from the robot, and send that angle
# D: end
#       Data has been sent, our program is done running

#in stateA, the program will initialize
#as long as it initializes, the next state becomes searching for marker
def stateA():
    return stateB
    
    
#in this state we just see if we have detected a marker in main or not
def stateB():
        if ids is not None: #if we have found anything
            #then we move to the next state, where we are calculating the angle it is at. The datapath then also sends this data
            return stateC
        else:
            return stateB #if it isn't found, we maintain the same state

#this state ends with calculating the angle, meaning that regardless it will send the angle. 
def stateC():
    return stateD
    
#this state just sends the angle and transitions us to the next one
def stateD():
    return stateE
    
#this state just signifies that the program is done. It won't ever really do anything, since it won't be called after it is set
def stateE():
    #nothing
    return None
    
state_dictionary = { #making the state dictionary for my code. 
    stateA : "Initialize",
    stateB : "Searching",
    stateC : "Detected",
    stateD : "Sending",
    stateE : "PI Finished"
}
state = stateA #make this start in stateA for the actuall program. state D for i2c send check



#these are the global variables we need. This is how I am currently passing data between states
ids = None
corners = None
angle = None
#my concern about this right now is just that maybe the data isn't being kept across states. Global variables? Pass them into the states? 
#I have decided to initialize outside the state machine? I'm not sure about this but otherwise I'd not know how to get the variables

#we need to initialize our program and stuff
#Arduino Iniialization
ARD_ADDR = 8
i2c_arduino = SMBus(1)


#thread Initialization. Not necessary as long as we aren't using LCD
#Q = queue.Queue()

#threadLCD = threading.Thread(target = LCD_Display, args = ())
#threadLCD.start()
#time.sleep(1) #gives it a second tin initialize

#camera initialization. Needs to be accessible in all states though
cap = cv2.VideoCapture(0) #initializes camera channel
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) #set width 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) #set height
cap.set(cv2.CAP_PROP_EXPOSURE, -14)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #we're considering 50 of the 6x6 aruco markers
parameters = aruco.DetectorParameters() #sets parameters (for later use)
time.sleep(0.5) #allows time for initialization
# last_angle = 40 #this isn't necessary for now

#these are the global variables we need. This is how I am currently passing this data between states
ids = None
corners = None
angle = None

input("Press any key to continue:") #uncomment this for potential speed up

while state is not stateE:
        
    #we check to see what state we are in, and based on that, do stuff
    #comment this in for debugging
    #print(state_dictionary[state]) 
    if state is stateA:
        #right now it is initialized outside the machine, so this isn't necessary
        '''
        #we need to initialize our program and stuff
        #Arduino Iniialization
        ARD_ADDR = 8
        i2c_arduino = SMBus(1)


        #thread Initialization. Not necessary as long as we aren't using LCD
        #Q = queue.Queue()

        #threadLCD = threading.Thread(target = LCD_Display, args = ())
        #threadLCD.start()
        #time.sleep(1) #gives it a second tin initialize

        #camera initialization. Needs to be accessible in all states though
        cap = cv2.VideoCapture(0) #initializes camera channel
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) #set width 
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) #set height
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #we're considering 50 of the 6x6 aruco markers
        parameters = aruco.DetectorParameters() #sets parameters (for later use)
        time.sleep(0.5) #allows time for initialization
        # last_angle = 40 #this isn't necessary for now
        '''
        #all that will happen here is telling the arduino to start looking/spinning
        success = 0 #sending a 0, since our program can run!
        offset_success = 0
        i2c_arduino.write_byte_data(ARD_ADDR, offset_success, success)
        #then it will put is in the detecting state
        
        
    #if we are in this state, we have initialized the program and are therefore actively searching for a marker
    elif state is stateB:
        ret,frame = cap.read()
        if not ret:
                print("error capturing frame") #this just checks if we successfully found it
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #convert to grayscale
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #search for ids
        
        #uncomment this if you want to output the image of what's happening. It slows us down though.
        
        
        
    #if we are in this state, we must have successfully found at least one marker
    #we can therefore access the global variable 'corners' to determine the marker's location
    #might be important to tell the arduino that it has been detected, so there isn't weird time stuff.
    elif state is stateC: 
        #remember that corners has the first 2 indices to select the marker (in case of 1 marker 0 0)
        # the third index stores which corner, starting at top left and going clockwise tll bottom left. Fourth stores x (0) or y (1)
        #xWidth is the top right x - top left x + bottom right x - bottom left x.
        #I decided to add in the top and bottom and take the average in case there is any tangential distortion
        xWidth = (corners[0][0][1][0] - corners[0][0][0][0] + corners[0][0][2][0] - corners[0][0][3][0])/2

        #yWidth should be bottom left y minus top left y + bottom right y 0 top right y /2
        yWidth = (corners[0][0][3][1] - corners[0][0][0][1] + corners[0][0][2][1] - corners[0][0][1][1])/2

        #this is all for distance detection, which isn't necessary yet. It also might be flawed, considering angle stuff
        '''
        #Here we are using the equation F = (P x D)/W
        #P is going to be the width, D is the distance we place it at initially, in centimeters (1 meter or 100 cm), W is the size (14.4 cm)
        Fx = (xWidth * 100)/(5)
        Fy = (yWidth * 100)/(5)
        #print(f"Focal length x is: {Fx}") #x is 650, y is 635
        #print(f"Focal length y is: {Fy}")
        '''

        #to calculate the average of each of the corner coordinate sets
        xCenter = np.mean(corners[0][0][:, 0]) #mean of all corner x coordinates
        yCenter = np.mean(corners[0][0][:, 1]) #mean of all corner y coordinates

        #based on our xCenter, we get The aruco angle
        angle = Get_Angle(xCenter)
        
        #that was all this state needed to do
        
    #if we are in this state, angle has been calculated, so we just need to send to arduino
    elif state is stateD:


        #got this code from this website: https://stackoverflow.com/questions/33451800/decimal-to-binary-half-precision-ieee-754-in-python
        #np.float16(angle) converts our calculated angle to IEEE 754 half precision format
        #.view("H") treats the float we made as an unsigned integer of 16 bits. Otherwise it may be treated differently
        #bin() converts that unsigned integer into a string of its 16 bits, with 0b at the start
        #taking the result from [2:] gets rid of the first bits, which are going to be 0b
        #.zfill(16) pads the string with zeros to make sure its 16 bits long.
        # .zfill(16) is necessary because If the number was represented with many 0s at the start, bin() would shrink them down likely
        IEEE_754_Value = bin(np.float32(angle).view("I"))[2:].zfill(32)
        #print(IEEE_754_Value)

        byte1 = IEEE_754_Value[:8]
        byte2 = IEEE_754_Value[8:16]
        byte3 = IEEE_754_Value[16:24]
        byte4 = IEEE_754_Value[24:]
        #print(byte1)
        #print(byte2)
        #print(byte3)
        #print(byte4)
        #this converts the 8 characters into an integer, using 2 for the base
        byte1_val = int(byte1, 2) 
        byte2_val = int(byte2, 2) 
        byte3_val = int(byte3, 2)
        byte4_val = int(byte4, 2)
                
        data = [byte1_val, byte2_val, byte3_val, byte4_val] #the data to be sent
        offset_data = 1 #we want to write to the second register I believe, and the other thing (spin start) will go to the first. Just arbitrary though
        #send the data
        try:
            #ask arduino to take encoder reading
            i2c_arduino.write_i2c_block_data(ARD_ADDR, offset_data, data)
        except IOError:
            print("Could not write data to the arduino")
        print(angle)

    
    #this is our state transition. Various states have different things they consider, so instead of setting up their inputs I make them global
    new_state = state()
    state = new_state

#this is what happens after we enter into stateE, since we exit the loop
cap.release()
cv2.destroyAllWindows()
