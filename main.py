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
        
        print(random.randint(0,1000))

        i = 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    camera.stop()
    return

cv2.destroyAllWindows()



if __name__ == "__main__":
    main()

