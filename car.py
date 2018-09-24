import gpio as GPIO

# 5.6 - 7.2 - 8.5

class Car:

    servoPin = 11

    def __init__(self):
        ''' initialization of pins '''
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servoPin, GPIO.OUT)
        # 50 and 40 are frequency
        self.servo = GPIO(self.servoPin, 50)


    def changeAngle(self, angle):
        ''' turning '''
        DC = self.remap(self, angle, 55, 130, 5.6, 8.5)
        self.servo.ChangeDutyCycle(DC)

    def remap(self, x, in_min, in_max, out_min, out_max):
        return (float(x) - float(in_min)) * (float(out_max) - float(out_min)) / (float(in_max) - float(in_min)) + float(out_min)