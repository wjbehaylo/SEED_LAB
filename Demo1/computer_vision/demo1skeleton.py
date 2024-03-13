import cv2
import numpy as np
from cv2 import aruco
from time import sleep

dictionary = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# Load calibration images
calibration_images = [cv2.imread(f'calibration_image_{i}.jpg') for i in range(1, 11)]

# Define the pattern size (e.g., chessboard)
pattern_size = (5, 5)

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Find chessboard corners in each calibration image
for frame in calibration_images:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Find the corners
    ret, corners = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # If found, add object points and image points
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Load the image
image = cv2.imread('aruco_image.jpg')

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect ArUco markers
corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)

# Estimate pose of ArUco markers
rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_size, mtx, dist)

# Undistort image points
undistorted_corners = cv2.undistortPoints(corners, mtx, dist)

# Calculate distance and angle to marker (assuming only one marker is detected)
distance = np.linalg.norm(tvecs[0])
angle = np.degrees(np.arctan2(tvecs[0][0][0], tvecs[0][0][2]))

print("Distance to marker:", distance)
print("Angle to marker:", angle)
