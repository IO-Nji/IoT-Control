# Battery Class
from machine import Pin,ADC # type: ignore

# BATTERY VOLTAGE Measuring Pin
vBat = 29

class Battery:
    
    def __init__(self):
        self.vPin = ADC(Pin(vBat))
    def readVoltage(self):   
        voltage = self.vPin.read_u16()
        return voltage
    


