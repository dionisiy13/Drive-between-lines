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
varProcess = 3
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



        hsv_min =  np.array([54,118,84])
        hsv_max = np.array([99,255,255])
        binary_mask = cv2.inRange(hsv_image, hsv_min, hsv_max)
	cv2.imshow("23233", binary_mask)

        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(binary_mask, low_threshold, high_threshold)


        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 90  # angular resolution in radians of the Hough grid
        threshold = 15  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 20  # minimum number of pixels making up a line
        max_line_gap = 15  # maximum gap in pixels between connectable line segments
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

	cY1 += 30
        # search the near lines from the center
        nearLines = getTheNearestLine(arrayLines, cX1, cY1, line_image)

        # draw that lines
        cv2.line(line_image,
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


        error = etalonValue - 250
	#print("error - ")
	#print (error)
        kp = (nearLines[1] - nearLines[0])/2
	#print("kp - ")
	#print(kp)
        kp = float(30)/float(kp)
        kp = float(kp) + 0.3
        output = float(kp) * float(error)
        output = output + 95
	print("output before - ")
	print(kp)
        if (output > 130):
            output = 130
        if (output < 55):
            output = 55

        output = (130 - output) + 55



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

