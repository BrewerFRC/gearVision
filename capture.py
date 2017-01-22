import pistream
import cv2
imageCounter = 0

# Setup PiCamera for capture
IMG_WIDTH = 240
IMG_HEIGHT = 180

try:
        video = pistream.PiVideoStream((IMG_WIDTH,IMG_HEIGHT),format="bgr").start()

        while raw_input("Enter to capture or 'q' to quit") != "q":
                image = video.read()
                cv2.imwrite("captures/image" + str(imageCounter) + ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
                print "saved image "+str(imageCounter)
                imageCounter += 1

finally:
        video.stop()
	

