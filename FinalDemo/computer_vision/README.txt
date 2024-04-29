# Computer Vision

Experimenting.py:
- file used for basic state machine frame to integrate our seperate distance and angle calculation files

FINALstateMachine.py:
- file used for the final demo, includes a state machine that interacts with the arduino in order to meet
  the requirements of the final demo

cameraThreading.py:
- test file used to implement camera threading and test camera parameters to find the most optimal settings

demo3CV.py:
- original state machine file used to get a basic test done, was discarded at a later time in order to have a
  cleaner file for debugging purposes

finalDemoCV.py:
- another test file used for debugging, near identical to demo3CV.py

getCameraM.py:
- file that was a conglomerate of online resources use to calculate the camera specific matrix from calibration images.
  Neccesary step for accurate distance calculation

stateBdemo3.py:
- test file used for ensuring state B went to the closest detected aruco marker

takeCalibrationImages.py:
- file used to take an N length string of images in order to train the camera matrix.
