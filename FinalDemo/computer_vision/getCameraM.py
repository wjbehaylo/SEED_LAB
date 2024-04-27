import numpy as np
from time import sleep
import cv2
import glob
from numpy import savetxt
from numpy import genfromtxt

resolution = (1250,690)
#critera = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Prepare object points, assuming the chessboard pattern is on a plane (z=0)
objp = np.zeros((9*6, 3), np.float32)
objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
# Arrays to store object points and image points from all calibration images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load calibration images
calibration_images = glob.glob('/home/seedlab/CalibrationImages/*.jpg')  # Change the path as needed

# Iterate over calibration images
for fname in calibration_images:
    # Load the image
    img = cv2.imread(fname)
    # Convert image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
    # If corners are found, add object points and image points
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        print("done")
    else:
        print("no corners")


ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, resolution, None, None)

print("Calculated matrix")
print(camera_matrix)
print("Distortion Coeff")
print(distortion_coefficients)
