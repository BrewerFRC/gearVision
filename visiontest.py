import numpy
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from networktables import NetworkTable
import pistream
import sys


# if running from command line, passing any value as a parameter will
# stop the image windows from being displayed.
if len(sys.argv) > 1:  #python filename is always in arg list, so look for 2 or more
    showImages = False
else:
    showImages = True


# Define a range of HSV values for thresholding - green light reflection
#robot shop
#LOWER_HSV = numpy.array([57,206,30]) 
#UPPER_HSV = numpy.array([105,255,255])
#Gym
#LOWER_HSV = numpy.array([30,23,50]) 
#UPPER_HSV = numpy.array([104,255,220])
#At Pine Tree
#LOWER_HSV = numpy.array([49,26,112]) 
#UPPER_HSV = numpy.array([94,255,226])
# Camera with Exposure in Night mode
#LOWER_HSV = numpy.array([45,162,130]) 
#UPPER_HSV = numpy.array([78,255,255])
LOWER_HSV = numpy.array([43,100,110])
UPPER_HSV = numpy.array([99,255,255])

#These constants control goal detection values
GOAL_RATIO = 25.0 / 55.0    #Retroreflective tape Height/Width (pixel counted from camera view)
GOAL_MINIMUM_AREA = 50   #The smallest number of pixel area that a goal should be
GOAL_MAXIMUM_AREA = 550
GOAL_RATIO_ERROR = 0.20     #Allowable error from ratio
GOAL_CENTER = 129           #X position within camera frame where we need to center goal
GOAL_CENTER_ERROR = 4       #Allowable error from center, in pixels.

#Parameters for robot turning speed
TURN_SCALE = 0.01           #Multiplier to determine turn rate, given error from center in pixels that will determine the robot turn rate
TURN_MAX_RATE = 0.9        #Fastest turn rate allowed
TURN_MIN_RATE = 0.55      #was .7 Slowest turn rate allowed

# Setup PiCamera for capture
IMG_WIDTH = 240
IMG_HEIGHT = 180
video = pistream.PiVideoStream((IMG_WIDTH,IMG_HEIGHT)).start()
time.sleep(2)  #Give camera a chance to stablize


# Connect to roboRio via NetworkTable
NetworkTable.setIPAddress("10.45.64.2")
NetworkTable.setClientMode()
NetworkTable.initialize()

table = NetworkTable.getTable("visionTracking")


# Based on the error from target center, in pixels, this function will
# calculate the appropriate rate at which the robot should turn.
def calcTurnRate(error):
    # Determine magnitude of turn using the scale factor
    absRate = abs(error) * TURN_SCALE
    # Constrain turn rate within acceptable Min and Max bounds
    if absRate < TURN_MIN_RATE:
        absRate = TURN_MIN_RATE
    else:
        if absRate > TURN_MAX_RATE:
            absRate = TURN_MAX_RATE
    # Apply direction to turn based on the sign of the error
    if error >= 0:
        return absRate
    else:
        return -absRate


# Process images continuously, outputting a comamnd to the robot each cycle
def process():
    # Init contour measurements for picture saving
    h = 0
    w = 0
    center = 0
    imageCounter = 0  #Counter for filename of saved images

    kernel = numpy.ones((3,3),
                        numpy.uint8)  #used to do dialate
    
    while True:

        # Grab frame from PiCamera
        img_bgr = video.read()

        # Resize to a smaller image for better performance
        img_bgr = cv2.resize(img_bgr, (IMG_WIDTH,IMG_HEIGHT),interpolation = cv2.INTER_LINEAR)
        if showImages:
            cv2.imshow('img_bgr',img_bgr)

        # Blur the image
        #img_blur = cv2.blur(img_bgr,(5,5))
        #img_blur = img_bgr
        #if showImages:
            #cv2.imshow("img_blur",img_blur)

        # Create an HSV variant of the image
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        # Create a binary threshold based on the HSV range (look for green reflective tape)
        img_thresh = cv2.inRange(img_hsv, LOWER_HSV, UPPER_HSV)
        if showImages:
            cv2.imshow("img_thresh",img_thresh)

        # Dilate them image to fill holes
        img_dialate = cv2.dilate(img_thresh, kernel, iterations=1)
        if showImages:
            cv2.imshow("img_dialate",img_dialate)

        # Find all of the contours in binary image
        _, contours, _ = cv2.findContours(img_dialate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Now we'll evaluate the contours.
        # We'll assume that no goal is visible at first.
        # The value of 'turn' will indicate what action the robot should take.
        # 0=no goal, do nothing, 999=aligned with goal, shoot, +/-x.xxx=turn rate to move to target
        turn = 0
        if len(contours) > 0:
            # print "Count:",len(contours)
            #Check all contours, or until we see one that looks like a goal
            for c in contours:
                # Is the contour big enough to be a goal?
                area = cv2.contourArea(c) # area of rectangle that is filled
                #print area
                if area > GOAL_MINIMUM_AREA:
                    # Is 5the h/w ratio close to that of the goal ratio?
                    x,y,w,h = cv2.boundingRect(c)
                    r = cv2.minAreaRect(c)
                    print format_number(r[0][0]), format_number(r[0][1]), format_number(r[1][0]), format_number(r[1][1]), format_number(r[2])  
                    

                    #print w,h,w/h
                    #print w, h, (x+w/2)
                    ratio = float(h) / float(w)
                    ratioError = ratio - GOAL_RATIO
                    if abs(ratioError) <= GOAL_RATIO_ERROR:
                        # This must be a goal, find its center point
                        center = x + (w/2)
                        centerError = center - GOAL_CENTER
                        # Is the goal alignment within allowable limits?
                        if (abs(centerError) <= GOAL_CENTER_ERROR):
                            #Yes, so command a shot
                            turn = 999
                        else:
                            #No, need to issue a turn command
                            turn = calcTurnRate(centerError)
                       #print h,w,area,center,turn
                        #print "w",w,"h",h,"c",center,"t",turn
                        break
        else:
            print "No contours found"
            turn = 0

        table.putNumber("targetTurn", turn)
        
        # Check to see if robot wants us to save a picture
        try:
            takePicture = table.getNumber("takePicture")
        except:
            takePicture = 0
            
        try:
            if takePicture == 1:
                print "Saving picture"
                # Print width, height ,center, turn on image
                text = "w"+str(w)+" h"+str(h)+" c"+str(center)+" t"+str(turn)
                cv2.putText(img_bgr,text,(1,150),cv2.FONT_HERSHEY_SIMPLEX,.35,(255,255,255),1)
                # Save image
                cv2.imwrite("/home/pi/vision/captures/image" + str(imageCounter) + ".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
                imageCounter += 1
                print "Image saved"
                table.putNumber("takePicture",0)
        except:
            print "Failed to take picture"
            raise
            table.putNumber("takePicture",0)

        # Wait 1ms for keypress. q to quit.
        if cv2.waitKey(1) &0xFF == ord('q'):
            break
        
def format_number(x):
    return '{:6.2f}'.format(float(x))

 
    
# Process until user exit (q or ctrl-c)
try:
    process()
# Always clean up before exit
finally:
    video.stop()
    cv2.destroyAllWindows()

     

