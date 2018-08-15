import cv2
import numpy as np
import os
import time
from functions import *
from imutils.video.pivideostream import PiVideoStream
import imutils
from pprint import pprint
from picamera.array import PiRGBArray
from picamera import PiCamera

# kalman settings
varVolt = 10
varProcess = 5
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

def main():
    global Xe

    print("init..")
    i = 0

    print("camera init..")
    camera = PiVideoStream().start()
    time.sleep(4.0)


    print("start!")
    #for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    while 1:

        #blnFrameReadSuccessfully, imgOriginal = capWebcam.read()
        # imgOriginal = frame.array

        imgOriginal = camera.read()
        imgOriginal = imutils.resize(imgOriginal, 500, 300)


        image = imgOriginal
        (h1, w1) = image.shape[:2]
        (cX1, cY1) = (w1 // 2, h1 // 2)

        hsv_image = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2HSV)



        hsv_min =  np.array([49,88,46])
        hsv_max = np.array([104,255,91])
        binary_mask = cv2.inRange(hsv_image, hsv_min, hsv_max)
	cv2.imshow("dsd", binary_mask)
        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(binary_mask, low_threshold, high_threshold)


        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 90  # angular resolution in radians of the Hough grid
        threshold = 15  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 50  # minimum number of pixels making up a line
        max_line_gap = 200  # maximum gap in pixels between connectable line segments
        line_image = np.copy(image) * 0  # creating a blank to draw lines on
        lines = []
        # Run Hough on edge detected image
        # Output "lines" is an array containing endpoints of detected line segments
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),min_line_length, max_line_gap)

        if lines is None:
            print("Fuck, sorry my lord, but I did not found any line( But I am trying........")
            continue

        arrayLines = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                arrayLines.append([x1,y1,x2,y2])


        # search the near lines from the center
        nearLinesCenter = getTheNearestLine(arrayLines, cX1, cY1, line_image)
        nearLinesTop = getTheNearestLine(arrayLines, cX1, cY1 - 10, line_image)
        nearLinesBotton = getTheNearestLine(arrayLines, cX1, cY1+10, line_image)

        # draw that lines
        cv2.line(line_image,
                 (nearLinesCenter[0], nearLinesCenter[2]),
                (nearLinesCenter[1], nearLinesCenter[2]),
                (0, 255, 0), 1)

        cv2.line(line_image,
                 (nearLinesTop[0], nearLinesTop[2]-10),
                 (nearLinesTop[1], nearLinesTop[2]-10),
                 (0, 255, 0), 1)

        cv2.line(line_image,
                 (nearLinesBotton[0], nearLinesBotton[2]+10),
                 (nearLinesBotton[1], nearLinesBotton[2]+10),
                 (0, 255, 0), 1)

        nearLineAvarage0 = int((nearLinesCenter[0] + nearLinesTop[0] + nearLinesBotton[0]) / 3)
        nearLineAvarage1 = int((nearLinesCenter[1] + nearLinesTop[1] + nearLinesBotton[1]) / 3)
        nearLineAvarage2 = int((nearLinesCenter[2] + nearLinesTop[2] + nearLinesBotton[2]) / 3)

	

        nearLinesCenter = [nearLineAvarage0,nearLineAvarage1, nearLineAvarage2]

	
        cv2.line(line_image,
                 (nearLineAvarage0, nearLineAvarage2),
                 (nearLineAvarage1, nearLineAvarage2),
                 (0, 0, 255), 1)

        # center
        cv2.line(line_image, (cX1, cY1-10), (cX1, cY1+10), (0, 255, 0), 2)
        cv2.line(line_image, (cX1 - 10, cY1), (cX1 + 10, cY1), (0, 255, 0), 2)

        needToControl = cX1
	
        # center control
        centerControl = int((nearLinesCenter[1] - nearLinesCenter[0]) / 2) + nearLinesCenter[0]
        if (i == 0):
            Xe = centerControl
        centerControl = [kalman(centerControl), nearLinesCenter[2]]


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

        cv2.line(line_image,
                 (nearLineAvarage0, nearLineAvarage1 - 10),
                 (nearLineAvarage0, nearLineAvarage1 + 10),
                 (255, 255, 255), 2)
        cv2.line(line_image,
                 (nearLineAvarage0 - 10, nearLineAvarage1),
                 (nearLineAvarage0 + 10, nearLineAvarage1),
                 (0, 0, 0), 2)

        #pprint("Need to control - " + str(needToControl))
        #pprint("Etalon value - " + str(etalonValue))

        # send to arduino
	if (nearLinesCenter[0] == 0): 
            transferToArduino(130) 

        elif (nearLinesCenter[1] == 668): 
            tranferToArduino(55) 
	else:

            error = etalonValue - 250
            
	    #print("error - ")
            #print (error)
	    widthBetweenLines = (nearLinesCenter[1] - nearLinesCenter[0])/2
            kp = kalman(widthBetweenLines)
            #print("kp - ")
            #print(kp)
            kp = float(75)/float(kp)
            kp = float(kp) 
            output = float(kp) * float(error)
            output = output + 95
            print("output before KP - ")
            print(kp)
            if (output > 130):
                output = 130
            if (output < 55):
                output = 55

            output = (130 - output) + 55
	    lastError = error


            transferToArduino(int(output))
            print(output)

        cv2.line(imgOriginal, (cX1, cY1 - 10), (cX1, cY1 + 10), (0, 255, 0), 2)
        cv2.line(imgOriginal, (cX1 - 10, cY1), (cX1 + 10, cY1), (0, 255, 0), 2)

        for item in arrayLines:
            cv2.line(line_image, (item[0], item[1]), (item[2], item[3]), (0, 0, 255), 1)

        lines_edges = cv2.addWeighted(imgOriginal, 0.8, line_image, 1, 0)


        #cv2.imshow("binar mask", binary_mask)

        cv2.imshow("original", lines_edges)

        #rawCapture.truncate()
        #rawCapture.seek(0)

        i = 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    camera.stop()
    return

cv2.destroyAllWindows()



if __name__ == "__main__":
    main()

