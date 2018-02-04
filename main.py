import cv2
import numpy as np
import os
import functions
from pprint import pprint
from picamera.array import PiRGBArray
from picamera import PiCamera

def main():

    #capWebcam = cv2.VideoCapture("files/video_2.mp4")
    capWebcam = PiCamera()
    camera.resolution = (1200, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(900,480))

    if capWebcam.isOpened() == False:

        os.system("pause")
        return

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #while cv2.waitKey(1) != 27 and capWebcam.isOpened():
        #blnFrameReadSuccessfully, imgOriginal = capWebcam.read()
         blnFrameReadSuccessfully, imgOriginal = frame.array

        if not blnFrameReadSuccessfully or imgOriginal is None:
            os.system("pause")
            break

        # crop the useful part of image
        image = imgOriginal
        (h, w) = image.shape[:2]
        (cX, cY) = (w // 2, h // 2)
        x = int(cX / 2)
        y = int(cY / 2)

        newImage = image[y:cY + y, x:cX + x]

        (h1, w1) = newImage.shape[:2]
        (cX1, cY1) = (w1 // 2, h1 // 2)

        # make img more readable for openCV
        gray = cv2.cvtColor(newImage, cv2.COLOR_BGR2GRAY)

        # settings for detect lines
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
        low_threshold = 50
        high_threshold = 250
        edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 90  # angular resolution in radians of the Hough grid
        threshold = 15  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 50  # minimum number of pixels making up a line
        max_line_gap = 20  # maximum gap in pixels between connectable line segments
        line_image = np.copy(newImage) * 0  # creating a blank to draw lines on

        # Run Hough on edge detected image
        # Output "lines" is an array containing endpoints of detected line segments
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)

        arrayLines = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                arrayLines.append([x1,y1,x2,y2])
        # search the near lines from the center
        nearLines = functions.getTheNearestLine(arrayLines, cX1, cY1, line_image)

        # draw that lines
        cv2.line(line_image, (nearLines[0], nearLines[2]), (nearLines[1], nearLines[2]), (0, 255, 0), 1)
        cv2.line(newImage, (nearLines[0], nearLines[2]), (nearLines[1], nearLines[2]), (0, 255, 0), 1)

        # center
        cv2.line(line_image, (cX1, cY1-10), (cX1, cY1+10), (0, 255, 0), 2)
        cv2.line(line_image, (cX1 - 10, cY1), (cX1 + 10, cY1), (0, 255, 0), 2)

        needToControl = cX1

        # center control
        centerControl = int((nearLines[1] - nearLines[0]) / 2) + nearLines[0]
        centerControl = [functions.kalman(centerControl), nearLines[2]]

        etalonValue = centerControl[0]

        cv2.line(line_image, (centerControl[0], centerControl[1] - 10), (centerControl[0],centerControl[1] + 10), (255, 255, 255), 2)
        cv2.line(line_image, (centerControl[0] - 10, centerControl[1]), (centerControl[0] + 10, centerControl[1]), (255, 255, 255), 2)

        pprint("Need to control - " + str(needToControl))
        pprint("Etalon value - " + str(etalonValue))


        cv2.line(newImage, (cX1, cY1 - 10), (cX1, cY1 + 10), (0, 255, 0), 2)
        cv2.line(newImage, (cX1 - 10, cY1), (cX1 + 10, cY1), (0, 255, 0), 2)

        for item in arrayLines:
            cv2.line(line_image, (item[0], item[1]), (item[2], item[3]), (0, 0, 255), 1)

        lines_edges = cv2.addWeighted(newImage, 0.8, line_image, 1, 0)

        cv2.imshow("lines", line_image)
        cv2.imshow("original", newImage)

    cv2.destroyAllWindows()
    return

if __name__ == "__main__":
    main()


