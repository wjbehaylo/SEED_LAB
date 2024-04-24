#EENG 350: SEED LAB
#FSM for Demo 2 CV

from smbus2 import SMBus
import time
import cv2
import cv2.aruco as aruco
import numpy as np

def Generate_IEEE_vector(value):   
    value_32_bits =bin(np.float32(value).view("I"))[2:].zfill(32)
    byte1 = value_32_bits[:8]
    byte2 = value_32_bits[8:16]
    byte3 = value_32_bits[16:24]
    byte4 = value_32_bits[24:]
    byte1_val = int(byte1, 2) 
    byte2_val = int(byte2, 2) 
    byte3_val = int(byte3, 2)
    byte4_val = int(byte4, 2)
    
    return [byte1_val, byte2_val, byte3_val, byte4_val]

def stateA():
    return stateB
        
#in this state we are consistently detecting markers, waiting for 'end' from arduino
def stateB():
    global transition
    if transition is True: 
        transition = False 
        time.sleep(0.1)
        return stateC
    else:
        return stateB

#this state ends when arduino sends us that circle is beginning
def stateC():
    global transition 
    global count

    if transition is True: 
        transition = False 
        #count = count +1 #we increment the number of wait states we have done
        #if (count == 7): 
         #   return stateE 
        #else: 
        return stateD 
        
    elif transition is False:
        return stateC 
    
#this state is the one in which we are rotating around the circle, looking for markers
#here we are both detecting and sending, so if we get to the state transition and we have detected a marker, we can assume it was sent as well
def stateD():
    global ids
    global distance
    global angle
    global transition

    ids = None
    distance = None
    angle = None

    if transition is True:
        transition = False
        time.sleep(.1)
        return stateC 
    else:  
        return stateD
    
#this state just signifies that the program is done. It won't ever really do anything, since it won't be called after it is set
def stateE():
    return None
    
state_dictionary = { #making the state dictionary for my code. 
    stateA : "Initialize",
    stateB : "360 searching",
    stateC : "waiting",
    stateD : "circle searching",
    stateE : "PI Finished"
}

#Aruco variables
ACK_360_DONE = 2
ACK_90_Done = 3
ARD_ADDR = 8
i2c_arduino = SMBus(1)

#Pi variables
global transition
global ids
global corners
global distance
global angle
global count
global received
ids = None
corners = None
distance = None
angle = None
transition = False
count = 1
received = 0

prev_len_ids = 0 #default it to 0 at the beginning
offset = 0
state = stateD 

#calculated camera matrix
camera_matrix = np.array([[1100.16914, 0, 822.431875],
                          [0, 1099.874175, 501.381015],
                          [0, 0, 1]])
dist_coeffs = np.array([-.13395956, 1.44725569, 0.01324846, .04205741, -2.61323609])  
marker_size = 0.05 #side length of aruco

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_EXPOSURE, -14)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()
#time.sleep(.5)

while state is not stateE:
    
    
    if state is stateA:
        success = 0 
        offset_success = 0
        i2c_arduino.write_byte_data(ARD_ADDR, offset_success, success) # tell the arduino to move
        
    elif state is stateB:
        try:
            received=i2c_arduino.read_byte_data(ARD_ADDR, offset) #since the arduino is just sending us something if it should be, we just need to see if it sends anything
        except IOError:
            pass
        if (received == ACK_360_DONE):
            transition = True
        else:
            transition = False
        #transition is dropped to False (if it is true) in the state transition logic
        
        #at the beginning of the big while loop, we determine if arduino has sent anything.
        if transition is False: #so we only do this searching if transition is false
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #convert to grayscale
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #search for ids
            
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

                prev_len_ids = len(ids)

            elif ids is None:
                prev_len_ids = 0
                
        #if transition is true, then we are just exiting this state. 

        
    #if we are in this state, we just need to be waiting for the arduino to send any information
    elif state is stateC:
        transition = False
        received = 0
        try:
            received = i2c_arduino.read_byte_data(ARD_ADDR, offset)
        except IOError:
            pass

        if (received == 4):
            received = 0
            transition = True # if a four is received we allow ourselves to go to the next state which is D
            
        
        
    #if we are in this state, the robot is moving in a circle and we will be looking to detect new markers
    elif state is stateD:
        #potentially clear variables
        ids = None
        distance = None
        angle = None

        while True:
            ret,frame = cap.read()
            if not ret:
                print("error capturing frame")
        
            corners, ids, _ = aruco.detectMarkers(frame,aruco_dict,parameters=parameters)   
            #cap.release()
            #cap.open(0)
            #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            #cap.set(cv2.CAP_PROP_EXPOSURE, -14)
            #cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            #aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
            #parameters = aruco.DetectorParameters()
            #time.sleep(1)
            
            
            if ids is not None and len(ids)>0: # this conditional is causing all of our issues
                transition = True
            
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], marker_size, camera_matrix, dist_coeffs)
                xCenter = np.mean(corners[0][0][:, 0]) #mean of all corner x coordinates

                angle = (xCenter -640)/640 * 30.7
                distance = np.linalg.norm(tvecs[0])

                dist_array = Generate_IEEE_vector(distance)
                angle_array= Generate_IEEE_vector(angle) 
                data = np.concatenate((angle_array, dist_array)) #the data to be sent, a combination of the two
                offset_data = 1

                try:
                    i2c_arduino.write_i2c_block_data(ARD_ADDR, offset_data, data)
                    print("distance: ", distance)
                    print("angle: ", angle)
                    cap.release()
                    cap.open(0)
                    time.sleep(.1)
                except IOError:
                    print("Could not write data to the arduino")
        
            break #this break may need to be moved

    #this is our state transition. Various states have different things they consider, so instead of setting up their inputs I make them global
    new_state = state()
    state = new_state

#this is what happens after we enter into stateE, since we exit the loop

