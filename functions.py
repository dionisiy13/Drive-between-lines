import cv2
import time
import smbus2 as smbus
from pprint import pprint
from smbus2 import SMBusWrapper

address = 0x04
bus = smbus.SMBus(1)

def eachPixelOfLine(line,centerY):
    xStart = line[2] if line[0] > line[2] else line[0]
    xEnd = line[2] if line[0] < line[2] else line[0]
    yStart = line[3] if line[1] > line[3] else line[1]
    yEnd = line[3] if line[1] < line[3] else line[1]

    reverse = True if line[1] > line[3] else False
	
    xLen = xEnd - xStart
    yLen = yEnd - yStart


    if (xLen > yLen):
        xPixels = range(xStart, xEnd)
        step = float(yLen) / float(xLen)
        step = round(step, 2)

        yAcc = yStart
	
        for x in xPixels[1:]:
            yAcc = float(yAcc) + float(step)
            if (abs(int(round(yAcc)) - centerY) < 5):
                yield [x, int(round(yAcc))]

    else:
	
        yPixels = range(yStart,yEnd)
        step = float(xLen) / float(yLen)
        step = round(step, 2)

        xAcc = xStart
        if (reverse):
            for y in reversed(yPixels[1:]):
                xAcc = float(xAcc) + float(step)
                if (y == centerY):
                    yield [int(round(xAcc)), y]
        else:
            for y in yPixels[1:]:
                xAcc = float(xAcc) + float(step)
                if (y == centerY):
                    yield [int(round(xAcc)), y]


def getTheNearestLinesNew(image, centerY, centerX):
    width = centerX * 2
    left = 0
    right = width
    #print(centerY)
    i = 0
    for x in reversed(range(width/2)):
        if (i==0):
	    i = 1
	    continue
	if (image[centerY, x] == (255.0)):
            left = x
	    #print(x)
	    break
    for x in range(width/2, width):
        if (i==1):
	    i = 2
	    continue
        if (image[centerY, x] == (255.0)):
            right = x
            break
    return [left, right, centerY]

def getTheNearestLine(lines, centerX, centerY, img):

    nearestRigthX = 0
    nearestLeftX = 0
    for line in lines:
	
        for linePixel in eachPixelOfLine(line, centerY):
	    
            x = linePixel[0]
            y = linePixel[1]
         
            if (x > centerX):
                if (centerX - x < nearestRigthX):
                    nearestRigthX = x
            else:
                if (centerX - x > nearestLeftX):
                    nearestLeftX = x
		    #pprint(x)

    return [nearestLeftX, nearestRigthX, centerY]


def transferToArduino(number):
     #with SMBusWrapper(1) as bus:
         # Write a byte to address 80, offset 0
      #  bus.write_byte_data(address, 0, number)
	#msg = i2c_msg.write(address, [65, 66, 67, 68])
        #bus.i2c_rdwr(msg)

    try:
	#time.sleep(0.25)
        bus.write_byte(address, number)
        #time.sleep(0.25)
    except IOError:
	print "Exception \n"
        time.sleep(0.2)


# def transferToArduino(number):
#     return true
#     try:
#         ser = serial.Serial("/dev/ttyUSB0", 9600)
#         ser.flush()
#         ser.write(str(number))
#         ser.write("\n")
#         ser.flush()
#         time.sleep(0.01)
#         return True
#     except:
#         print("No available arduino")
#         return True
