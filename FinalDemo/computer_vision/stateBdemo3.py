# Here is our state B code
#
# The goal of this code is to continuously detect aruco markers
# and send them to the arduino, without sending the same marker multiple
# times.

import numpy as np
import cv2
from cv2 import aruco

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
parameters = cv2.aruco.DetectorParameters()
cap = cv2.VideoCapture(0)

# Define camera matrix
camera_matrix = np.array([[1302.18142, 0, 757.661523],
                          [0, 1277.72316, 333.541675],
                          [0, 0, 1]])

# Define distortion coefficients (if any)
dist_coeffs = np.array([-.0501863, .35496149, -.00359878,.02920639,-.68738736])  # You may need to adjust the shape depending on your camera

# Marker size in meters
marker_size = 0.025  

# Function to calculate distance to ArUco marker
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
prev_len_ids = 0

while True:
    ret,frame = cap.read()
    if not ret:
        break
    
# actual detection
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        if len(ids) > prev_len_ids: #only do if we have detected a new marker
            for i in range(len(ids)):
                # Estimate pose of the marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)

            # Calculate distance to the marker
                distance = np.linalg.norm(tvecs[0])
                #angle = np.degrees(np.arctan2(tvecs[i][0][0], tvecs[i][0][2]))
            
            # Display distance
                print(f"Distance to marker with ID {ids[i][0]}: {distance:.5f} meters")
                #print(f"Angle to marker with ID {ids[i][0]}: {angle:.5f} degrees")
                
        prev_len_ids = len(ids)
        
    elif ids is None:
        prev_len_ids = 0

cap.release()
cv2.destroyAllWindows()
