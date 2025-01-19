# Battery Class
from machine import Pin,ADC
class Battery:
    def __init__(self, battPin):
        self.vPin = ADC(Pin(battPin))
    def readVoltage(self):   
        voltage = self.vPin.read_u16()
        return voltage
    


