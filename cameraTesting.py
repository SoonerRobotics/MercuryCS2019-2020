import cv2

cv2.namedWindow("preview")
vc = cv2.VideoCapture(1) #Set this to 1 in order to use the webcamera if you have one built in

# In theory, this is ideal, but it breaks the program:
#vc.set(3, 320)
#vc.set(4, 240)

#Retrieving the properties for the frame size
width = vc.get(3)   # float
height = vc.get(4) # float

width = int(width)
height = int(height)

oldDimensions = (width, height) #For upscaling after downscaling, so that the size of the image appears constant

scale_percent = 25 #Modify this to change the quality of the image, 100 equals no change
width = width * scale_percent / 100
height = height * scale_percent / 100

width = int(width)
height = int(height)

newDimensions = (width, height)

if vc.isOpened():
    rval, frame = vc.read()
else:
    rval = False

while rval:
    # cv2.imshow("preview", frame)
    rval, frame = vc.read()

    #Changing image quality
    downscale = cv2.resize(frame, newDimensions, interpolation = cv2.INTER_AREA)
    resized = cv2.resize(downscale, oldDimensions, interpolation = cv2.INTER_AREA)

    #Changing image color
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

    cv2.imshow("preview", gray)

    key = cv2.waitKey(20)
    if key == 27: #Break on ESC
        break

cv2.destroyWindow("preview")