import cv2
import numpy as np
import os
import time
from functions import *
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
    pprint(cv2.__version__);

    '''capWebcam = cv2.VideoCapture("files/video4.mp4")

    if capWebcam.isOpened() == False:
        os.system("pause")
        return
    '''
    i = 0
    #while cv2.waitKey(1) != 27 and capWebcam.isOpened():

    camera = PiCamera()
    camera.resolution = (800, 200)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(800,200))

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        #blnFrameReadSuccessfully, imgOriginal = capWebcam.read()
        imgOriginal = frame.array


        # make the frame more bright
        lab = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        limg = cv2.merge((cl,a,b))
        #imgOriginal = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)


        image = imgOriginal
        newImage = image
        (h1, w1) = image.shape[:2]
        (cX1, cY1) = (w1 // 2, h1 // 2)


        gray = cv2.cvtColor(newImage, cv2.COLOR_BGR2GRAY)



        # settings for detect lines
        kernel_size = 5
        #blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)



        blank_image = np.zeros_like(newImage)
        hsv_image = cv2.cvtColor(imgOriginal, cv2.COLOR_RGB2HSV)

        hsv_min =  np.array([5,60,60])
        hsv_max = np.array([80,255,255])
        binary_mask = cv2.inRange(hsv_image, hsv_min, hsv_max)


        masked_image = draw_binary_mask(binary_mask, hsv_image)
       ## edges_mask = cv2.Canny(masked_image, 280, 360)

        #cv2.imshow("original", masked_image)
        #cv2.waitKey()  # make img more readable for openCV


        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(masked_image, low_threshold, high_threshold)


        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 90  # angular resolution in radians of the Hough grid
        threshold = 15  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 20  # minimum number of pixels making up a line
        max_line_gap = 15  # maximum gap in pixels between connectable line segments
        line_image = np.copy(newImage) * 0  # creating a blank to draw lines on
        lines = []
        # Run Hough on edge detected image
        # Output "lines" is an array containing endpoints of detected line segments
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)
		
        arrayLines = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                arrayLines.append([x1,y1,x2,y2])


        # search the near lines from the center
        nearLines = getTheNearestLine(arrayLines, cX1, cY1, line_image)

        # draw that lines
        cv2.line(line_image,
                 (nearLines[0], nearLines[2]),
                 (nearLines[1], nearLines[2]),
                 (0, 255, 0), 1)
        cv2.line(newImage,
                 (nearLines[0], nearLines[2]),
                 (nearLines[1], nearLines[2]),
                 (0, 255, 0), 1)

        # center
        cv2.line(line_image, (cX1, cY1-10), (cX1, cY1+10), (0, 255, 0), 2)
        cv2.line(line_image, (cX1 - 10, cY1), (cX1 + 10, cY1), (0, 255, 0), 2)

        needToControl = cX1

        # center control
        centerControl = int((nearLines[1] - nearLines[0]) / 2) + nearLines[0]
        if (i == 0):
            Xe = centerControl
        centerControl = [kalman(centerControl), nearLines[2]]

        # for control
        etalonValue = centerControl[0]

        cv2.line(line_image,
                 (centerControl[0], centerControl[1] - 10),
                 (centerControl[0],centerControl[1] + 10),
                 (255, 255, 255), 2)
        cv2.line(line_image,
                 (centerControl[0] - 10, centerControl[1]),
                 (centerControl[0] + 10, centerControl[1]),
                 (255, 255, 255), 2)

        #pprint("Need to control - " + str(needToControl))
        #pprint("Etalon value - " + str(etalonValue))

        # send to arduino
        transferToArduino(etalonValue)

        cv2.line(newImage, (cX1, cY1 - 10), (cX1, cY1 + 10), (0, 255, 0), 2)
        cv2.line(newImage, (cX1 - 10, cY1), (cX1 + 10, cY1), (0, 255, 0), 2)

        for item in arrayLines:
            cv2.line(line_image, (item[0], item[1]), (item[2], item[3]), (0, 0, 255), 1)
            cv2.line(binary_mask, (item[0], item[1]), (item[2], item[3]), (0, 0, 255), 1)

        lines_edges = cv2.addWeighted(newImage, 0.8, line_image, 1, 0)


        #cv2.imshow("binar mask", binary_mask)

        cv2.imshow("original", lines_edges)
        i = 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    return

cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

