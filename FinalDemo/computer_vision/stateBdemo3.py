import numpy as np
import cv2
from cv2 import aruco

#Standard initialization we've done for every assignment
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
parameters = cv2.aruco.DetectorParameters()
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#This is the camera matrix and distortion coefficients found by running
#the getCameraM.py and takeCalibrationImages.py files
camera_matrix = np.array([[914.28571, 0, 640],
                          [0, 914.28571, 360],
                          [0, 0, 1]])
dist_coeffs = np.array([-.0501863, .35496149, -.00359878, .02920639, -.68738736])  

# Marker size in meters (Area or side length? seems to be right when
# equal to .025 for some reason)
marker_size = 0.025  

#The function used to not only undistort the image, but also find the distance
def calculate_distance(marker_corners, marker_size, camera_matrix, dist_coeffs):
    #Uses the mean of pixels for the x and y side length of the aruco in order to get an acurate tell
    #of what the side length really is. For example, the as the x component pixels are reduced, (i.e
    #the aruco is rotated horizontally), the y side length pixels will increase as it gets closer to 
    #the camera, and vice versa, so by taking a mean we get the actual side length
    marker_length_pixels = np.mean([np.linalg.norm(marker_corners[0][0] - marker_corners[0][1]),
                                     np.linalg.norm(marker_corners[0][1] - marker_corners[0][2])])
    
    #This seems to be high level math but heres the best explanation I could find on the internet
    #"cv2.undistortPoints finds homogeneous coordinates for points in the image. This function removes 
    #lens distortion and unprojects the points so that they are in dimensionless coordinates. This 
    #function accepts the following arguments: an array of 2D points in the image, a 3x3 camera matrix, 
    #a set of distortion coefficients, an object to store the result, and rectification and projection 
    #matrices, which are used in the stereo vision and aren't relevant now. The last three arguments are 
    #optional. cv2.undistortPoints returns the set of undistorted and unprojected points.
    #The points returned by cv2.undistortPoints are ideal—their coordinates are dimensionless and aren't distorted by lenses." 
    undistorted_points = cv2.undistortPoints(marker_corners, camera_matrix, dist_coeffs)
    
    #this is decided by the camera matrix, predefined from the calibration images
    focal_length_x = camera_matrix[0, 0]
    distance = (marker_size * focal_length_x) / marker_length_pixels
    return distance

prev_len_ids = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Actual detection
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        if len(ids) > prev_len_ids:  # Only process if we have detected a new marker
            for i in range(len(ids)):
                # Calculate distance to the marker using the calculate_distance function, if this
                #is inaccurate or we also want angle calculation, we will need to use the 
                #estimate pose function
                distance = calculate_distance(corners[i], marker_size, camera_matrix, dist_coeffs)
                print(f"Distance to marker with ID {ids[i][0]}: {distance:.5f} meters")
                
        prev_len_ids = len(ids)
    #need to reset prev_len_ids so if one goes out of frame before the next, the distance will update.
    elif ids is None:
        prev_len_ids = 0

cap.release()
cv2.destroyAllWindows()
