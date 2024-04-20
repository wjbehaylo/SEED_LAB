#EENG 350: SEED LAB
#FSM for Demo 2 CV

from smbus2 import SMBus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
import cv2.aruco as aruco
import numpy as np


def Get_Angle(xCenter):
    angle = (xCenter -640)/640 * 30.7 #this 30.7 will have to change
    return angle


#this function has inputs of:
# the angle the aruco is detected at(in degrees, with left being negative)
# distance that the marker was detected at(in centimeters probably)
# radius of the circle (centimeters, probably)
#it will return the angle and distance that we sent to controls, to rotate and move before starting circle again
#
def Get_R_Position(theta_sa, R, d_sa):
    #arcsin returns -pi/2 to pi/2 radians, which will be converted to degrees (multiplied by 180/pi)
    
    #theta_ra is the angle between the point on the circle we go to, and the line from robot to aruco
    theta_ra = np.arcsin(R/d_sa) *(180/np.pi)
    
    #theta_m is the amount we will have to rotate so we are pointing towards x_r
    
    #proof/example
    #theta_ra will be positive, while theta_sa will likely be negative,
    #so if we detect it at -30 degrees, and determine that the angle to the point is 15 degrees, 
    #this will give us -30+15 = -15, which is correct.
    #because we always rotate left and detect markers on the left (negative side), this will work
    theta_m = theta_sa + theta_ra
    
    #d_sr is the distance we will travel once we have rotated theta_m. To hop in the circle
    d_sr = np.cos(theta_ra)*d_sa
    
    #returns angle and distance in a tuple
    return (theta_m, d_sr)
    

#


#States:
# A: initialize
#       where the program starts, starting camera, establishing I2C connection with Arduino

# B: detect markers in 360 degrees
#       robot is spinning in a circle, we need to get aruco markers
#       we will continue to detect until we receive from the arduino, indicating that 360degrees is done
#       

# C: wait
#       Data has been sent, we now wait for the arduino to tell us that we are starting to circle/look for markers
#       We record the number of wait states, and when the seventh one ends we go to 'done'

# D: search
#       We constantly look for markers as we move around the circle
#       when they are found we send the data and move back to the wait state

# E: done
#       Execution completed. Time to turn off camera and stuff

#in stateA, the program will initialize
#as long as it initializes, the next state becomes detect markers in 360 degrees
def stateA():
    return stateB
    
    
#in this state we are consistently detecting markers, waiting for 'end' from arduino
def stateB():
        global stateDone
        if stateDone is True: #if  arduino is done spinning
            stateDone = False #then that state is no longer done since we are transitioning into the next one
            #then we move to the next state, waiting for circling to start
            return stateC
        else:
            return stateB #if arduino

#this state ends when arduino sends us that circle is beginning
def stateC():
    #we modify these, so we need to make sure they are globally defined
    global stateDone 
    global C_count
    if stateDone is True: #if the arduino is done moving
        stateDone = False #state is no longer done
        #then we are either moving to state D(searching) or state E(done)
        C_count = C_count +1 #we increment the number of wait states we have done
        if (C_count == 7): #if we have done 7 of these approaches
            return stateE #we are done with the program
        else: #if we haven't approached 7 times
            return stateD #we keep searching
        
    else: #if the state is not done (arduino hasn't sent)
        return stateC #we are still waiting
    
#this state is the one in which we are rotating around the circle, looking for markers
#here we are both detecting and sending, so if we get to the state transition and we have detected a marker, we can assume it was sent as well
def stateD():
    if ids is not None: #if we have detected a marker
        #we first clear the list of detected markers
        '''
        Need to do more here for transition logic
        '''
        return stateC #we now wait for controls to move

    else: #if we haven't detected a marker, 
        return stateD #we continue to search
    
#this state just signifies that the program is done. It won't ever really do anything, since it won't be called after it is set
def stateE():
    #nothing
    return None
    
state_dictionary = { #making the state dictionary for my code. 
    stateA : "Initialize",
    stateB : "360 searching",
    stateC : "waiting",
    stateD : "circle searching",
    stateE : "PI Finished"
}

#we need to initialize our program and stuff
#Arduino Iniialization
ARD_ADDR = 8
i2c_arduino = SMBus(1)



#camera initialization. Needs to be accessible in all states though
cap = cv2.VideoCapture(0) #initializes camera channel
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) #set width 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) #set height
cap.set(cv2.CAP_PROP_EXPOSURE, -14) #set high exposure to detect markers better
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #we're considering 50 of the 6x6 aruco markers
parameters = aruco.DetectorParameters() #sets parameters (for later use)
time.sleep(0.5) #allows time for initialization
# last_angle = 40 #this isn't necessary for now

#these are the global variables we need. This is how I am currently passing data between states
ids = None
corners = None
angle = None
stateDone = False
#I store the data globally so that our program can access it when in different states

state = stateA #make this start in stateA for the actual program. state D for i2c send check

input("Press any key to continue:") #uncomment this for potential speed up

while state is not stateE:
        
    #we check to see what state we are in, and based on that, do stuff
    #comment this in for debugging
    #print(state_dictionary[state]) 
    
    #this line here is to determine if we have received anything from the arduino, since it is important
    offset = 0 #offset that we will be receiving from...?
    received=i2c_arduino.read_byte_data(ARD_ADDR, offset) #since the arduino is just sending us something if it should be, we just need to see if it sends anything
    if received is not None: #if the arduion has sent anything
        stateDone = True
    #we can assume that if it doesn't send anything, stateDone will remain false. But just in case, we'll set it
    else:
        stateDone = False
        
    if state is stateA:
        #right now it is initialized outside the machine, so this isn't necessary
        
        #all that will happen here is telling the arduino to start looking/spinning
        success = 0 #sending a 0, since our program can run!
        offset_success = 0
        i2c_arduino.write_byte_data(ARD_ADDR, offset_success, success)
        #we have told arduino to start spinning. we will now (soon) move
        
        
    #if we are in this state, we have initialized the program and are therefore spinning around
    elif state is stateB:
        '''
        This one is not done and needs to be revisited with logic and distance/angle detection
        '''
        #at the beginning of the big while loop, we determine if arduino has sent anything.
        if stateDone is False: #so we only do this searching if stateDone is false
            ret,frame = cap.read()
            if not ret:
                    print("error capturing frame") #this just checks if we successfully found it
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #convert to grayscale
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #search for ids
        #if stateDone is true, then we are just exiting this state. 
    
        
    #if we are in this state, we just need to be waiting for the arduino to send any information
    elif state is stateC: 
        
        
        
        #remember that corners has the first 2 indices to select the marker (in case of 1 marker 0 0)
        # the third index stores which corner, starting at top left and going clockwise tll bottom left. Fourth stores x (0) or y (1)
        #xWidth is the top right x - top left x + bottom right x - bottom left x.
        #I decided to add in the top and bottom and take the average in case there is any tangential distortion
        xWidth = (corners[0][0][1][0] - corners[0][0][0][0] + corners[0][0][2][0] - corners[0][0][3][0])/2

        #yWidth should be bottom left y minus top left y + bottom right y 0 top right y /2
        yWidth = (corners[0][0][3][1] - corners[0][0][0][1] + corners[0][0][2][1] - corners[0][0][1][1])/2

        #this is all for distance detection, which isn't necessary yet. It also might be flawed, considering angle stuff


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
