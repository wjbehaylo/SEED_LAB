from time import sleep
import numpy as np
import cv2

# this function makes camera parameters 640 x 480, captures an image, converts that image to grayscale, and displays it
def camera_640_480_capture():
        '''
        print("Select a width and height combination by entering the integer above the desired combination")
        print("option: 1       2       3      4      5      6      7      8\nwidth:  1280    1280    960    800    800    640    640    424\nheight: 800     720     544    600    448    480    360    240"
        option = int(input())
        if(option
        '''
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        ret, image = camera.read()
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #this line makes it grayscale
        image = cv2.Canny(image, 10, 200) #this line adds color edges
 
        if not ret:
                print("Could not capture image from camera!")
                quit()
        else:
                cv2.imshow('Image',image)
                cv2.waitKey(0)


fileName = input("File Name: ")
	
# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)
	
# Let the camera warmup
sleep(0.1)

camera_640_480_capture()

''' Given DEMO CODE

# Get an image from the camera stream
ret, image = camera.read()


        
if not ret:
	print("Could not capture image from camera!")
	quit()
else:
        cv2.imshow('Image',image)
        cv2.waitKey(0)
        save = int(input("Would you like to save the image?\nYes(1)\nNo(2)\n"))
        cv2.destroyAllWindows()

        if(save == 1):
              # Save the image to the disk
                print("Saving image "+fileName)
                try:
                        cv2.imwrite(fileName,image)
                except:
                        print("Could not save "+fileName)
                        pass
'''
	
		
	
