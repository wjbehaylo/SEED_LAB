import cv2
from cv2 import aruco
import numpy as np
from time import sleep

print(""+cv2.__version__)

def detect_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    detectorParams = aruco.DetectorParameters()
    refineParams = aruco.RefineParameters()
    detector = aruco.ArucoDetector(aruco_dict, detectorParams, refineParams)
    corners, ids, _ = detector.detectMarkers(gray)

cap = cv2.VideoCapture(0)
detect = ""

while True:
    ret,frame = cap.read()

    if not ret:
        print("error capturing frame")
        break
    detect_markers(frame)

    cv2.imshow('Frame',frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
