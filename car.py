import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)

servo = GPIO.PWM(11, 50)
servo.start(7.0)

class Car:

    servoPin = 11
    servo = 1

    def __init__(self):
        ''' initialization of pins '''
        #GPIO.setmode(GPIO.BOARD)
        #GPIO.setup(self.servoPin, GPIO.OUT)
        # 50 and 40 are frequency
        
    def changeAngle(self, angle):
        ''' turning '''
        DC = self.remap(angle, 55, 130, 5.6, 8.5)
        servo.ChangeDutyCycle(DC)

    def remap(self, x, in_min, in_max, out_min, out_max):
        return (float(x) - float(in_min)) * (float(out_max) - float(out_min)) / (float(in_max) - float(in_min)) + float(out_min)
