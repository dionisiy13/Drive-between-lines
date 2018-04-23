import time
import serial

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
        step = yLen / xLen
        step = round(step, 2)

        yAcc = yStart

        for x in xPixels[1:]:
            yAcc = yAcc + step
            if (int(round(yAcc)) == centerY):
                yield [x, int(round(yAcc))]

    else:
        yPixels = range(yStart,yEnd)
        step = xLen / yLen
        step = round(step, 2)

        xAcc = xStart
        if (reverse):
            for y in reversed(yPixels[1:]):
                xAcc = xAcc + step
                if (y == centerY):
                    yield [int(round(xAcc)), y]
        else:
            for y in yPixels[1:]:
                xAcc = xAcc + step
                if (y == centerY):
                    yield [int(round(xAcc)), y]




def getTheNearestLine(lines, centerX, centerY, img):

    nearestRigthX = 688
    nearestLeftX = 0
    for line in lines:

        for linePixel in eachPixelOfLine(line, centerY):

            x = linePixel[0]
            y = linePixel[1]
            if (y != centerY):
                continue
            if (x > centerX):
                if (centerX - x < nearestRigthX):
                    nearestRigthX = x
            else:
                if (centerX - x > nearestLeftX):
                    nearestLeftX = x

    return [nearestLeftX, nearestRigthX, centerY]



def transferToArduino(number):
    return true
    try:
        ser = serial.Serial("/dev/ttyUSB0", 9600)
        ser.flush()
        ser.write(str(number))
        ser.write("\n")
        ser.flush()
        time.sleep(0.01)
        return True
    except:
        print("No available arduino")
        return True