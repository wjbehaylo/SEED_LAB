#EENG 350: SEED LAB
#FSM for Demo 2 CV

from smbus2 import SMBus
import time
import cv2
import cv2.aruco as aruco
import numpy as np
from threading import Thread

class CameraThread(Thread):
    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_CONTRAST, 100)
        #self.cap.set(cv2.CAP_PROP_EXPOSURE, -2)
        #self.cap.set(cv2.CAP_PROP_SATURATION, 50)
        #self.cap.set(cv2.CAP_PROP_FPS, 60)
        
        if not self.cap.isOpened():
            
            
            
            print("Error: Could not open camera.")
            #self.camera_avaliable = False
            return

        self.frame = None
        self.running = False
        self.camera_available = True
        
    def run(self):
        if not self.camera_available:
            print("Exiting thread due to camera error.")
            return

        self.running = True
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Failed to read frame.")
                continue
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.frame = frame

        self.cap.release()
        print("Camera released.")

    #def detectMotionBlur(self, frame, threshold=25):
    #    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #    variance = cv2.Laplacian(gray, cv2.CV_64F).var()
    #    if variance < threshold:
    #       return True
    #    else:
    #       return False
        

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
        #    return stateE 
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
global all_distance
global all_angle
all_distance = np.array([])
all_angle = np.array([])
ids = None
corners = None
distance = None
angle = None
transition = False
count = 1
received = 0

prev_len_ids = 0 #default it to 0 at the beginning
offset = 0

state = stateA

camera_matrix = np.array([[917.48747702, 0, 579.38699741],
 [0, 843.05047474, 229.32305489],
 [0, 0, 1]])
dist_coeffs = np.array([[ .00552048554, .346335661, -.000255577484, .101832126, -.351282856]])

marker_size = 0.05 #side length of aruco

camera_thread = CameraThread()
camera_thread.start()

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()

while state is not stateE: 
    
    if state is stateA:
        success = 0 
        offset_success = 0
        try:
            i2c_arduino.write_byte_data(ARD_ADDR, offset_success, success) # tell the arduino to move
        except:
            pass
        
    elif state is stateB:
        while True:
            try:
                received=i2c_arduino.read_byte_data(ARD_ADDR, offset) #since the arduino is just sending us something if it should be, we just need to see if it sends anything
                if (received == 2):
                    transition = True
                    break
                else:
                    transition = False

            except IOError:
                pass
            
          
            if transition is False:
                if camera_thread.frame is not None:
                    frame = camera_thread.frame
                    corners, ids, _ = aruco.detectMarkers(frame,aruco_dict,parameters=parameters)  
          
                if ids is not None: # this conditional is causing all of our issues
                    for i in range(len(ids)):
                        _ , tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)
                        distance = np.linalg.norm(tvecs[0])*.7
                        xCenter = np.mean(corners[i][:, 0])
                        angle = (xCenter -320)/320 * 30.7
                        dist_array = Generate_IEEE_vector(distance)
                        angle_array= Generate_IEEE_vector(angle) 
                        data = np.concatenate((angle_array, dist_array))
                        offset_data = 1
                        
                        try:
                            i2c_arduino.write_i2c_block_data(ARD_ADDR, offset_data, data)
                            print("distance: ", distance)
                            print("angle: ", angle)
                        except IOError:
                            print("Could not write data to the arduino")
                            break
            
            
    #if we are in this state, we just need to be waiting for the arduino to send any information
    elif state is stateC:
        received = 0
        frame = camera_thread.frame
        try:
            received = i2c_arduino.read_byte_data(ARD_ADDR, offset)
        except IOError:
            pass
        #print(received)
        if (received == 6):
            received = 0
            transition = True # if a four is received we allow ourselves to go to the next state which is D
        else:
            transition = False
 
    #if we are in this state, the robot is moving in a circle and we will be looking to detect new markers
    elif state is stateD:
        while True:
            ids = None
            distance = None
            angle = None
            frame = None
            corners = None
            if camera_thread.frame is not None:
                frame = camera_thread.frame
                corners, ids, _ = aruco.detectMarkers(frame,aruco_dict,parameters=parameters)   
                
            if ids is not None and len(ids)>0: # this conditional is causing all of our issues
                transition = True
                
                _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], marker_size, camera_matrix, dist_coeffs)
                xCenter = np.mean(corners[0][0][:, 0]) #mean of all corner x coordinates

                angle = (xCenter -320)/320 * 30.7
                distance = np.linalg.norm(tvecs[0])*.85

                dist_array = Generate_IEEE_vector(distance)
                angle_array= Generate_IEEE_vector(angle) 
                data = np.concatenate((angle_array, dist_array)) #the data to be sent, a combination of the two
                offset_data = 1

                try:
                    i2c_arduino.write_i2c_block_data(ARD_ADDR, offset_data, data)
                    print("distance: ", distance)
                    print("angle: ", angle)
                    
                except IOError:
                    print("Could not write data to the arduino")
        
            break #this break may need to be moved

    #this is our state transition. Various states have different things they consider, so instead of setting up their inputs I make them global
    new_state = state()
    state = new_state

