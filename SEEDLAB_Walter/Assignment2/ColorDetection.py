import cv2
import numpy as np
from time import sleep

camera = cv2.VideoCapture(0)
sleep(0.1)

#uncomment to capture a new image
'''
ret, image = camera.read()

if not ret:
    print("Could not capture image from camera!")
else:
    cv2.imshow('Image',image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
'''


#uncomment to use saved image

image = cv2.imread('/home/seedlab/SEEDLAB_Walter/Assignment2/testImage.png')
if image is None:
    print("Could not read image")
else:
    cv2.imshow('Image',image)
    k = cv2.waitKey(0) #displays the image
    cv2.destroyAllWindows()


#this gets the shape of the image in coordinates and color values
y,x,c = image.shape

'''
myBGR = image[int(y/4), int(x/4)] #pixel from upper left quadrant of image
print(f"BGR {myBGR}")

#images are 3d matrixes, third dimension being the color channel (0 is blue, 1 is green and 2 is red). So the third dimension is a size 3 array
imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
myRGB = imageRGB[int(y/4), int(x/4)]
print(f"RGB {myRGB}")
'''

#H represents the hue (actual color), measured ccw from positive X axis
#S represents amount that color is expressed, so distance from center of wheel
#V represents value, so how much black is mixed with the color (high value is no black)
#We convert the image to HSV, and use that from now on, so that we can filter the colors
imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
myHSV = imageHSV[int(y/4), int(x/4)]
print(f"HSV {myHSV}")

#80 to 150 degrees, 10% and above saturation, 51-255 (20% and above) value
#these are the values of pixel HSV that are considered green (in my opinion)
uppergreen = np.array([75, 255, 255])
lowergreen = np.array([40, 25, 51])

#this gives us the mask, however the image will just be in black and white
mask = cv2.inRange(imageHSV, lowergreen, uppergreen)

#uncomment to show un morphologically transformed mask

cv2.imshow("mask", mask)


#THis is where we do morphology to make the mask more accurate.
#erosion makes it so pixels are only 1 if all bixels under kernel are 1
#dillution makes it so pixels are 1 if any pixels under kernel are 1
#opening is erosion followed by dilution, to get rid of small things then grow the og back to size, Good for noise
#closing is dilution then erosion, fills black points and stuff

#I think I am just going to open it to get rid of noise, then close it to fill it u
kernel = np.ones((4,4), np.uint8)


#first I'm gonna open it with a small kernel to get rid of some noise
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#uncomment to show opened mask

cv2.imshow("mask opened", mask)


kernel = np.ones((60,60), np.uint8)
#then once I've gotten rid of the noise I'm going to close it with a bigger kernel to try and bring in the shape
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
#uncomment to show closed mask

cv2.imshow("mask closed", mask)

#display the mask as well as the morphologically transformed masks for comparison
k = cv2.waitKey(0)
cv2.destroyAllWindows()



#uncomment to show mask w/color
'''
#shows the green portions of the image
result = cv2.bitwise_and(image, image, mask=mask)
cv2.imshow("result",result)
k = cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.waitKey(0)
'''

contour_green_visualize = image.copy() #I think this is where the contours display

#find countours works with white object, black background.
#contours_green is a python list of the XY coordinates of the contours
#CHAIN_APPROX_SIMPLE is the method of contour approximation.
#This one finds 2 points along the contour and saved them, making them lines
contours_green,_ = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

#contour_green_visualize is the base image
#contours_green is the xy coordinates of the contours,
#-1 represents drawing all indices of contours_green
# 255,0,0 represents that we want them to be drawn in blue
# 3 represents the thickness
cv2.drawContours(contour_green_visualize, contours_green, -1, (255,0,0), 3)

#uncomment to display original image with contours
'''
cv2.imshow("Contours", contour_green_visualize)
k = cv2.waitKey(0)
cv2.destroyAllWindows()
'''

#these are necessary for our box drawing
imageBox = image.copy() #copy of original image
imageCenters = np.empty((0,2)) # 2d
imageAreas = np.empty((0)) # 1d

#checking each contour against minimum area, here I just used 300
threshold = 300

#for each pixel in the list of contours,
for index,cnt in enumerate(contours_green):
    contour_area = cv2.contourArea(cnt) #not super sure how this works, must just connect the coords and find areas
    if contour_area > threshold:
        x,y,w,h = cv2.boundingRect(cnt) #gets starting pixel, width, height of the box
        center = int(x+w/2), int(y+h/2) #center of box
        imageAreas = np.append(imageAreas, contour_area) #adds the contour as an area
        imageCenters = np.vstack((imageCenters, center))
        cv2.rectangle(imageBox, (x,y), (x + w, y + h), (0,255,0), 2) #edges, color, width
        cv2.putText(imageBox, 'Green', (x, y-10), cv2.FONT_ITALIC, 0.9, (0,255,0), 2)
        cv2.putText(imageBox, '+', center, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2) 
cv2.imshow("Big Contours", imageBox)
cv2.waitKey(0)
cv2.destroyAllWindows()

