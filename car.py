import gpio as GPIO

class Car:

    servoPin = 11
    IN3 = 29
    IN4 = 31
    ENB = 13

    def __init__(self):
        ''' initialization of pins '''
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servoPin, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        # 50 and 40 are frequency
        self.servo = GPIO(self.servoPin, 50)
        self.motor = GPIO(self.ENB, 40)


    def straightDirect(self):
        '''make staight direct'''
        GPIO.output(self.IN3, 1)
        GPIO.output(self.IN4, 0)

    def reverseDirect(self):
        ''' make reverse direct '''
        GPIO.output(self.IN3, 0)
        GPIO.output(self.IN4, 1)


    def changeAngle(self, angle):
        ''' turning '''
        DC = 1./18.*(angle)+2
        self.servo.ChangeDutyCycle(DC)

    def speed(self, speed):
        ''' control speed '''
        self.motor.ChangeDutyCycle(speed)

    def stop(self):
        ''' stop the car '''
        self.motor.ChangeDutyCycle(0)

    def activeStop(self):
        ''' active stop '''
        self.reverseDirect()
