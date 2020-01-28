import cv2, numpy

vc = cv2.VideoCapture(0) #Set this to 1 in order to use the webcamera if you have one built in

if vc.isOpened():
    rval, frame = vc.read()
else:
    rval = False

#Retrieving the properties for the frame size
width = vc.get(3)   # float
height = vc.get(4) # float

width = int(width)
height = int(height)

oldDimensions = (width, height) #For upscaling after downscaling, so that the size of the image appears constant

scale_percent = 50 #Modify this to change the quality of the image, 100 equals no change
width = width * scale_percent / 100
height = height * scale_percent / 100

width = int(width)
height = int(height)

newDimensions = (width, height)

#Changing image quality
downscale = cv2.resize(frame, newDimensions, interpolation = cv2.INTER_AREA)
#resized = cv2.resize(downscale, oldDimensions, interpolation = cv2.INTER_AREA)

#Changing image color
gray = cv2.cvtColor(downscale, cv2.COLOR_BGR2GRAY)

cv2.namedWindow("preview")

while rval:
    # cv2.imshow("preview", frame)
    rval, frame = vc.read()

    #Changing image quality
    downscale = cv2.resize(frame, newDimensions, interpolation = cv2.INTER_AREA)
    resized = cv2.resize(downscale, oldDimensions, interpolation = cv2.INTER_AREA)

    #Changing image color
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

    img_str = cv2.imencode('.jpg', gray)[1].tostring()

    nparr = numpy.frombuffer(img_str, numpy.uint8)
    frameParsed = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)

    cv2.imshow("preview", frameParsed)

    key = cv2.waitKey(20)
    if key == 27: #Break on ESC
        break

cv2.destroyWindow("preview")
