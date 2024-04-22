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
    #based on our xCenter, we get The aruco angle 
    angle = (xCenter -320)/320 * 30.7 #this 30.7 will have to change
    return angle


'''
#this is holden's calculate_distance function. For now, it is not in use, in preference of the tvec value for distance
def calculate_distance(marker_corners, marker_size, camera_matrix, dist_coeffs):
    # Assuming the marker is a square, so we can take average of width and height
    marker_length_pixels = np.mean([np.linalg.norm(marker_corners[0][0] - marker_corners[0][1]),
                                     np.linalg.norm(marker_corners[0][1] - marker_corners[0][2])])
    
    # Calculate focal length in pixels
    focal_length_x = camera_matrix[0, 0]
    focal_length_y = camera_matrix[1, 1]
    
    # Distance calculation
    distance = (marker_size * focal_length_x) / marker_length_pixels
    return distance
'''

#this function has inputs of:
# the angle the aruco is detected at(in degrees, with left being negative)
# distance that the marker was detected at(in centimeters probably)
# radius of the circle (centimeters, probably)
#it will return the angle and distance that we sent to controls, to rotate and move before starting circle again
#
def Get_R_Position(theta_sa, d_sa, R):
    #arcsin returns -pi/2 to pi/2 radians, which will be converted to degrees (multiplied by 180/pi)
    print("detected angle")
    print(theta_sa)
    print("detected distance")
    print(d_sa)
    #theta_asr is the angle between aruco, robot, and radius
    theta_asr = np.arcsin(R/d_sa) * 180/(np.pi)
    print("theta between aruco, robot, radius")
    print(theta_asr)
    d_sr = np.cos(theta_asr*np.pi/180) * d_sa #weird conversion I know, but np.cos takse in radians
    #returns angle and distance in a tuple

    if (abs(theta_sa)>theta_asr):
        theta_m = abs(theta_sa) - theta_asr
    else:
        theta_m = theta_asr -abs(theta_sa)
    
    return (theta_m, d_sr)

def Generate_IEEE_vector(value):
    #got this code from this website: https://stackoverflow.com/questions/33451800/decimal-to-binary-half-precision-ieee-754-in-python
        #np.float16(angle) converts our calculated angle to IEEE 754 half precision format
        #.view("H") treats the float we made as an unsigned integer of 16 bits. Otherwise it may be treated differently
        #bin() converts that unsigned integer into a string of its 16 bits, with 0b at the start
        #taking the result from [2:] gets rid of the first bits, which are going to be 0b
        #.zfill(16) pads the string with zeros to make sure its 16 bits long.
        # .zfill(16) is necessary because If the number was represented with many 0s at the start, bin() would shrink them down likely
        
    value_32_bits =bin(np.float32(value).view("I"))[2:].zfill(32)
    byte1 = value_32_bits[:8]
    byte2 = value_32_bits[8:16]
    byte3 = value_32_bits[16:24]
    byte4 = value_32_bits[24:]
    #print(byte1)
    #print(byte2)
    #print(byte3)
    #print(byte4)
    #this converts the 8 characters into an integer, using 2 for the base
    byte1_val = int(byte1, 2) 
    byte2_val = int(byte2, 2) 
    byte3_val = int(byte3, 2)
    byte4_val = int(byte4, 2)
    
    return [byte1_val, byte2_val, byte3_val, byte4_val]

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
            return stateB #if arduino isn't done spinning, state isn't done

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
    global ids #so that we can use ids being not None as our state transition logic
    
    if ids is not None: #if we have detected a marker
        ids = None #we need to change ids here, for the next iteration      
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
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) #set width. Holden's distance sets it to default
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) #set height. Default is 720 by 480 I Think
cap.set(cv2.CAP_PROP_EXPOSURE, -14) #set high exposure to detect markers better
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) #we're considering 50 of the 6x6 aruco markers
parameters = aruco.DetectorParameters() #sets parameters (for later use)
time.sleep(0.5) #allows time for initialization

#this is the camera matrix and distortion matrix that we calculated
camera_matrix = np.array([[1302.18142, 0, 757.661523],
                          [0, 1277.72316, 333.541675],
                          [0, 0, 1]])
dist_coeffs = np.array([-.0501863, .35496149, -.00359878,.02920639,-.68738736])  
# Marker size in meters, also necessary for distance calculation
marker_size = 0.025  

#these are the global variables we need. This is how I am currently passing data between states
ids = None
corners = None
angle = None
stateDone = False
C_count = 1 #make sure this is zero when we actually run it, 1 for starting at first
radius = 0.3575 #radius is 50 cm. This has units of meters, so we need 0.3575 meters
prev_len_ids = 0 #default it to 0 at the beginning
offset = 0

#these are the values of the messages that we want from the arduino

ACK_360_DONE = 2
ACK_90_Done = 3
ACK_MOVE_DONE = 4
#I store the data globally so that our program can access it when in different states
state = stateD #make this start in stateA for the actual program. state D for i2c send check

input("Press any key to continue:") #uncomment this for potential speed up

while state is not stateE:
        
    #we check to see what state we are in, and based on that, do stuff
    #comment this in for debugging
    print(state_dictionary[state]) 
    
     
    #now we enter the state conditionals
    if state is stateA:
        #right now it is initialized outside the machine, so this isn't necessary
        
        #all that will happen here is telling the arduino to start looking/spinning
        success = 0 #sending a 0, since our program can run!
        offset_success = 0
        i2c_arduino.write_byte_data(ARD_ADDR, offset_success, success)
        #we have told arduino to start spinning. we will now (soon) move
        
        
    #if we are in this state, we have initialized the program and are therefore spinning around
    elif state is stateB:
        #this line here is to determine if we have received anything from the arduino, since it is important
        received=i2c_arduino.read_byte_data(ARD_ADDR, offset) #since the arduino is just sending us something if it should be, we just need to see if it sends anything
        if (received == ACK_360_DONE): #if the arduion has sent anything
            stateDone = True
        #we can assume that if it doesn't send anything, stateDone will remain false. But just in case, we'll set it
        else:
            stateDone = False
        #stateDone is dropped to False (if it is true) in the state transition logic
        
        #at the beginning of the big while loop, we determine if arduino has sent anything.
        if stateDone is False: #so we only do this searching if stateDone is false
            ret,frame = cap.read()
            if not ret:
                print("error capturing frame") #this just checks if we successfully found it
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #convert to grayscale
            
            #corners has first index for which marker, 
            #second index for which corner? (0 is top left, 1 is top right, 2 is bottom right, 3 bottom left)
            #third index for x and y (supposedly), with 0 being x and 1 being y
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #search for ids
            
        #holden stateB code start:
            if ids is not None:
                if len(ids) > prev_len_ids: #only do if we have detected a new marker
                    for i in range(len(ids)):
                        # Estimate pose of the marker
                        _ , tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)

                        # Calculate distance to the marker, in meters
                        distance = np.linalg.norm(tvecs[0])
                        #angle = np.degrees(np.arctan2(tvecs[i][0][0], tvecs[i][0][2]))
                        xCenter = np.mean(corners[i][:, 0]) #mean of all corner x coordinates
                        #based on our xCenter, we get The aruco angle
                        angle = Get_Angle(xCenter)
                        
                        
                        # Display distance
                        # print(f"Distance to marker with ID {ids[i][0]}: {distance:.5f} meters") #we don't need to print this stuff
                        #print(f"Angle to marker with ID {ids[i][0]}: {angle:.5f} degrees")

                prev_len_ids = len(ids)

            elif ids is None:
                prev_len_ids = 0
                
        #if stateDone is true, then we are just exiting this state. 

        
    #if we are in this state, we just need to be waiting for the arduino to send any information
    #so nothing actually needs to happen here, all the logic is in the state function pointer itself
    elif state is stateC:
        #this line here is to determine if we have received anything from the arduino, since it is important
        received=i2c_arduino.read_byte_data(ARD_ADDR, offset) #since the arduino is just sending us something if it should be, we just need to see if it sends anything
        if (received == ACK_MOVE_DONE or received == ACK_90_Done): #if the arduino has sent anything
            stateDone = True
        #we can assume that if it doesn't send anything, stateDone will remain false. But just in case, we'll set it
        else:
            stateDone = False
        #I think we need to drop down the 'received flag' again in order to wait for a new thing to be sent
        
        
        
        
    #if we are in this state, the robot is moving in a circle and we will be looking to detect new markers
    elif state is stateD:
        
        
        ret,frame = cap.read()
        if not ret:
            print("error capturing frame")
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(frame,aruco_dict,parameters=parameters)
        #for now, we assume that the first marker detected will be the closest one/next. This may not be true
        #When debugging, be sure to stay aware of this
        #to calculate the average of each of the corner coordinate sets
        if ids is not None:
            xCenter = np.mean(corners[0][0][:, 0]) #mean of all corner x coordinates

            #based on our xCenter, we get The aruco angle
            angle = Get_Angle(xCenter)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], marker_size, camera_matrix, dist_coeffs)
            # Calculate distance to the marker, in meters
            distance = np.linalg.norm(tvecs[0])
            print("distance detected")
            print(distance)
            theta_m, d_sr = Get_R_Position(angle, distance, radius)
            print("angle to rotate")
            print(theta_m)
            print("distance to move")
            print(d_sr)
            dist_array = Generate_IEEE_vector(d_sr)
            angle_array= Generate_IEEE_vector(theta_m)
            
            data = np.concatenate((angle_array, dist_array)) #the data to be sent, a combination of the two
                    
            
            offset_data = 1 #we want to write to the second register I believe, and the other thing (spin start) will go to the first. Just arbitrary though
            #send the data
            try:
                #ask arduino to take encoder reading
                #This sends the bytes from last to first. 
                i2c_arduino.write_i2c_block_data(ARD_ADDR, offset_data, data)
            except IOError:
                print("Could not write data to the arduino")
            #we don't set ids to None here, since we need it to not be none for our state transition logic
            
    
    
    #we don't need anything for stateE, since we will never actually be in it
    
    
    #this is our state transition. Various states have different things they consider, so instead of setting up their inputs I make them global
    new_state = state()
    state = new_state

#this is what happens after we enter into stateE, since we exit the loop
cap.release()
cv2.destroyAllWindows()
