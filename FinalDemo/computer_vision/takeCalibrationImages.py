from time import sleep
import numpy as np
import cv2


fileName = "camra_calibration_img"
# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)
#camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#camera.set(cv2.CAP_PROP_EXPOSURE, -14)

# Let the camera warmup
sleep(1)

for i in range(200):
	j = str(i)
	sleep(.1)
	ret, image = camera.read()
	fileName = ""
	fileName = "/home/seedlab/CalibrationImages/"+j+".jpg"
	if not ret:
		print("Could not capture image from camera!")
		quit()
	else:
		# Save the image to the disk
		print("Saving image "+fileName)
		try:
			cv2.imwrite(fileName,image)
		except:
			print("Could not save "+fileName)
			pass
		
	
