import time
import smbus2 as smbus
from pprint import pprint
from smbus2 import SMBusWrapper


class TransferToArduino:

    bus = None
    address = 0x04

    def __init__(self):
        self.bus = smbus.SMBus(1)

    def say(self, number):
        try:
            self.bus.write_byte(self.address, number)
        except IOError:
            print 'Can not pass the data on arduino! \n'
            time.sleep(0.2)


