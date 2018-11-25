import cv2
import numpy as np
import os
import time
from TransferToArduino import *
from imutils.video.pivideostream import PiVideoStream
import imutils
from pprint import pprint
from picamera.array import PiRGBArray
from picamera import PiCamera
from DetectCenter import DetectCenter
from ProcessFrame import ProcessFrame
from ProportionalGain import ProportionalGain
from TransferToArduino import TransferToArduino
from Kalman import Kalman


def main():

    print("init..")
    i = 0
    print("camera init..")
    camera = PiVideoStream().start()
    time.sleep(4.0)
    print("start!")
    Kalman = Kalman()

    while 1:
        imgOriginal = camera.read()

        process_frame = ProcessFrame()

        image = imgOriginal.copy()
        (h1, w1) = image.shape[:2]
        (cX1, cY1) = (w1 // 2, h1 // 2)

        process_frame.set_top_border_color([62, 255, 77])
        process_frame.set_bottom_border_color([42, 70, 14])
        hsv_image = process_frame.apply_hsv_color(imgOriginal)


        binary_mask = process_frame.binaryzation(hsv_image)
        near_lines = DetectCenter.get_the_nearest_lines_new(binary_mask, cY1, cX1)
	pprint(near_lines)
        left_line = near_lines[0]
        right_line = near_lines[1]



        # draw that lines
        cv2.line(image,
                (left_line, cY1),
                (right_line, cY1),
                (0, 255, 0), 1)
		
	        
	# draw the center
        cv2.line(image, (cX1, cY1-10), (cX1, cY1+10), (0, 255, 0), 2)
        cv2.line(image, (cX1 - 10, cY1), (cX1 + 10, cY1), (0, 255, 0), 2)

        # calculated center
        center_control = int((right_line - left_line) / 2) + left_line

        if i == 0:
            Kalman.Xe = center_control

        # apply kalman`s filter
        center_control = Kalman.kalman(center_control)
	print(center_control)
	print("\n")
        # for control
        etalon_value = center_control

        # draw calculated center
        cv2.line(image,
                 (center_control, cY1 - 10),
                 (center_control, cY1 + 10),
                 (255, 255, 255), 2)
        cv2.line(image,
                 (center_control - 10, cY1),
                 (center_control + 10, cY1),
                 (255, 255, 255), 2)

        output = ProportionalGain.calculate(1.8, etalon_value, cX1, right_line - left_line)
        error = etalon_value - cX1

        arduino_transfer = TransferToArduino()
        if output is False:
	    a = 1
            # stop the car
            #arduino_transfer.say(1)
        else:
            # keep moving
	    a = 2
            #arduino_transfer.say(int(output))

        error = abs(error)

        if error > 0 and error < 10:
            color = (0, 255, 0)
        if error >= 10 and error <= 40:
            color = (0, 213, 255)
        if error > 40:
            color = (0, 0, 255)

        if output < 92:
            cv2.arrowedLine(image, (cX1-30, cY1-30), (cX1+30, cY1 - 30), color, 3)
        if output > 92:
            cv2.arrowedLine(image, (cX1+30, cY1-30), (cX1-30, cY1-30), color, 3)
        cv2.line(image, (cX1, cY1-10), (cX1, cY1+10), (0, 255, 0), 2)
        cv2.line(image, (cX1-10, cY1), (cX1+10, cY1), (0, 255, 0), 2)

        # show frames
        cv2.imshow("binary", binary_mask)
        cv2.imshow("original", image)

        i = i + 1
	
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
           
    camera.stop()
    return

cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

