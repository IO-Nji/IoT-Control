import time
from machine import Pin,I2C,SPI,PWM,ADC # type: ignore


# PIN DEFINITIONS
# SPI FOR DISPLAY
DC = 8
CS = 9
SCK = 10
MOSI = 11
RST = 12
BL = 25
# DISPLAY SPECIFICATIONS
width = 240
height = 240

# I2C FOR ACCELEROMETER
SCL = 7
SDA = 6

# BATTERY VOLTAGE MEASURING PIN
Vbat_Pin = 29

# Initialize variables to store the maximum and minimum values
max_acc_x = float('-inf')
max_acc_y = float('-inf')
max_acc_z = float('-inf')
min_acc_x = float('inf')
min_acc_y = float('inf')
min_acc_z = float('inf')

max_gyr_x = float('-inf')
max_gyr_y = float('-inf')
max_gyr_z = float('-inf')
min_gyr_x = float('inf')
min_gyr_y = float('inf')
min_gyr_z = float('inf')

class QMI8658(object):
    def __init__(self,address=0X6B):
        self._address = address
        self._bus = I2C(id=1,scl=Pin(SCL),sda=Pin(SDA),freq=100_000)
        bRet=self.WhoAmI()
        if bRet :
            self.Read_Revision()
        else    :
            return NULL
        self.Config_apply()

    def _read_byte(self,cmd):
        rec=self._bus.readfrom_mem(int(self._address),int(cmd),1)
        return rec[0]
    def _read_block(self, reg, length=1):
        rec=self._bus.readfrom_mem(int(self._address),int(reg),length)
        return rec
    def _read_u16(self,cmd):
        LSB = self._bus.readfrom_mem(int(self._address),int(cmd),1)
        MSB = self._bus.readfrom_mem(int(self._address),int(cmd)+1,1)
        return (MSB[0] << 8) + LSB[0]
    def _write_byte(self,cmd,val):
        self._bus.writeto_mem(int(self._address),int(cmd),bytes([int(val)]))
        
    def WhoAmI(self):
        bRet=False
        if (0x05) == self._read_byte(0x00):
            bRet = True
        return bRet
    def Read_Revision(self):
        return self._read_byte(0x01)
    def Config_apply(self):
        # REG CTRL1
        self._write_byte(0x02,0x60)
        # REG CTRL2 : QMI8658AccRange_8g  and QMI8658AccOdr_1000Hz
        self._write_byte(0x03,0x23)
        # REG CTRL3 : QMI8658GyrRange_512dps and QMI8658GyrOdr_1000Hz
        self._write_byte(0x04,0x53)
        # REG CTRL4 : No
        self._write_byte(0x05,0x00)
        # REG CTRL5 : Enable Gyroscope And Accelerometer Low-Pass Filter 
        self._write_byte(0x06,0x11)
        # REG CTRL6 : Disables Motion on Demand.
        self._write_byte(0x07,0x00)
        # REG CTRL7 : Enable Gyroscope And Accelerometer
        self._write_byte(0x08,0x03)

    def Read_Raw_XYZ(self):
        xyz=[0,0,0,0,0,0]
        raw_timestamp = self._read_block(0x30,3)
        raw_acc_xyz=self._read_block(0x35,6)
        raw_gyro_xyz=self._read_block(0x3b,6)
        raw_xyz=self._read_block(0x35,12)
        timestamp = (raw_timestamp[2]<<16)|(raw_timestamp[1]<<8)|(raw_timestamp[0])
        for i in range(6):
            xyz[i] = (raw_xyz[(i*2)+1]<<8)|(raw_xyz[i*2])
            if xyz[i] >= 32767:
                xyz[i] = xyz[i]-65535
        return xyz
    def Read_XYZ(self):
        xyz=[0,0,0,0,0,0]
        raw_xyz=self.Read_Raw_XYZ()  
        #QMI8658AccRange_8g
        acc_lsb_div=(1<<12)
        #QMI8658GyrRange_512dps
        gyro_lsb_div = 64
        for i in range(3):
            xyz[i]=raw_xyz[i]/acc_lsb_div#(acc_lsb_div/1000.0)
            xyz[i+3]=raw_xyz[i+3]*1.0/gyro_lsb_div
        return xyz

# Function to update the max and min values
def update_extremes(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z):
    global max_acc_x, max_acc_y, max_acc_z, min_acc_x, min_acc_y, min_acc_z
    global max_gyr_x, max_gyr_y, max_gyr_z, min_gyr_x, min_gyr_y, min_gyr_z

    if acc_x > max_acc_x:
        max_acc_x = acc_x
    if acc_y > max_acc_y:
        max_acc_y = acc_y
    if acc_z > max_acc_z:
        max_acc_z = acc_z

    if acc_x < min_acc_x:
        min_acc_x = acc_x
    if acc_y < min_acc_y:
        min_acc_y = acc_y
    if acc_z < min_acc_z:
        min_acc_z = acc_z

    if gyr_x > max_gyr_x:
        max_gyr_x = gyr_x
    if gyr_y > max_gyr_y:
        max_gyr_y = gyr_y
    if gyr_z > max_gyr_z:
        max_gyr_z = gyr_z

    if gyr_x < min_gyr_x:
        min_gyr_x = gyr_x
    if gyr_y < min_gyr_y:
        min_gyr_y = gyr_y
    if gyr_z < min_gyr_z:
        min_gyr_z = gyr_z


# Create QMI8658 object
qmi8658=QMI8658()             # Initialise gyro accl

# Main loop to read sensor data and update extremes
while True:
    # Read the accelerometer and gyroscope values
    xyz = qmi8658.Read_XYZ()
    acc_x, acc_y, acc_z = xyz[0], xyz[1], xyz[2]
    gyr_x, gyr_y, gyr_z = xyz[3], xyz[4], xyz[5]

    # Update the max and min values
    update_extremes(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z)

    # Print the current max and min values
    print(f"Acc Max: ({max_acc_x}, {max_acc_y}, {max_acc_z})")
    print(f"Acc Min: ({min_acc_x}, {min_acc_y}, {min_acc_z})")
    print(f"Gyr Max: ({max_gyr_x}, {max_gyr_y}, {max_gyr_z})")
    print(f"Gyr Min: ({min_gyr_x}, {min_gyr_y}, {min_gyr_z})")

    # Delay for a short period to control the sampling rate
    time.sleep(0.1)  # Adjust the delay as needed
