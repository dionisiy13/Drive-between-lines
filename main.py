import cv2
import numpy as np
import os
import time
from functions import *
from imutils.video.pivideostream import PiVideoStream
import imutils
import random
from pprint import pprint
from picamera.array import PiRGBArray
from picamera import PiCamera

# kalman settings
varVolt = 80
varProcess = 0.1
Pc = 0.0
G = 0.0
P = 1.0
Xp = 0.0
Zp = 0.0
Xe = 0


def kalman(val):
    global P
    global varProcess
    global Xe
    global Xp
    global Zp
    global G
    global Pc
    global varVolt
    Pc = P + varProcess
    G = Pc/(Pc + varVolt)
    P = (1 - G)*Pc
    Xp = Xe
    Zp = Xp
    Xe = G*(val-Zp)+Xp
    return int(Xe)


def draw_binary_mask(binary_mask, img):
        if len(binary_mask.shape) != 2:
            raise Exception('binary_mask: not a 1-channel mask. Shape: {}'.format(str(binary_mask.shape)))
        masked_image = np.zeros_like(img)
        for i in range(3):
            masked_image[:,:,i] = binary_mask.copy()
        return masked_image


def main():
    global Xe


    i = 0
    #while cv2.waitKey(1) != 27 and capWebcam.isOpened():

    '''capWebcam = cv2.VideoCapture("files/video4.mp4")

        if capWebcam.isOpened() == False:
            os.system("pause")
            return
        '''
    '''
    camera = PiCamera()
    camera.resolution = (800, 208)
    camera.framerate = 32
    camera.brightness = 63
    rawCapture = PiRGBArray(camera, size=(800,208))
    '''
    camera = PiVideoStream().start()
    time.sleep(4.0)


    #for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    while 1:

        #blnFrameReadSuccessfully, imgOriginal = capWebcam.read()
        # imgOriginal = frame.array

        imgOriginal = camera.read()
        imgOriginal = imutils.resize(imgOriginal, 500, 300)
        
        # make the frame more bright
        lab = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        limg = cv2.merge((cl,a,b))
        #imgOriginal = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)


        image = imgOriginal
        newImage = image
        (h1, w1) = (300, 500)
        #(cX1, cY1) = (w1 // 2, h1 // 2)
        (cX1, cY1) = (250, 150)


        gray = cv2.cvtColor(newImage, cv2.COLOR_BGR2GRAY)


        blank_image = np.zeros_like(newImage)
        hsv_image = cv2.cvtColor(imgOriginal, cv2.COLOR_RGB2HSV)

        hsv_min =  np.array([5,60,60])
        hsv_max = np.array([80,255,255])
        binary_mask = cv2.inRange(hsv_image, hsv_min, hsv_max)
        masked_image = draw_binary_mask(binary_mask, hsv_image)


        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(masked_image, low_threshold, high_threshold)


        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 90  # angular resolution in radians of the Hough grid
        threshold = 15  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 20  # minimum number of pixels making up a line
        max_line_gap = 15  # maximum gap in pixels between connectable line segments
        lines = []
        # Run Hough on edge detected image
        # Output "lines" is an array containing endpoints of detected line segments




        print(random.randint(0,100))

        i = 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    camera.stop()
    return

cv2.destroyAllWindows()



if __name__ == "__main__":
    main()

