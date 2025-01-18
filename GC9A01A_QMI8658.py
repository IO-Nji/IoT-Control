"""
Digital Display Panel - Round Display Demo Code
Author: Nji S. from https://nji.io
Date: 15th January 2025
This script demonstrates the use of a 240x240 LCD Round display (GC9A01A) with 16-bit color, 
along with a QMI8658 Accelerometer and Gyroscope. It is tested with the Waveshare RP2040 
1.28" IPS LCD Board with Accelerometer & Gyroscope.
The script includes:
- Initialization and configuration of the LCD display and QMI8658 sensor.
- Drawing various graphical elements on the display, including a clock face, 
    bar graphs, a 3D rotating cube, and a Cartesian graph.
- Displaying accelerometer and gyroscope data in real-time.
- Utility functions for drawing shapes, converting values, and displaying text.
Classes:
- LCD_1inch28: Handles the initialization and drawing operations for the LCD display.
- QMI8658: Manages the communication and data retrieval from the QMI8658 sensor.
- CartesianGraph: Draws a Cartesian graph to display sensor data.
- Cube: Draws and rotates a 3D cube based on gyroscope data.
- Point: Represents a point on the display.
Functions:
- draw_line: Draws a line on the display.
- colourConverter: Converts RGB888 to RGB565 color format.
- circle: Draws a filled circle on the display.
- ring: Draws a ring (circle without fill) on the display.
- rect: Draws a rectangle on the display.
- character: Draws a single character on the display.
- print_st: Prints a string of text on the display.
- print_yc: Centers and prints text on a specific line.
- drawHorizontalBargraph: Draws a horizontal bar graph on the display.
- drawRoundClockFaceNumbers: Draws numbers on the round clock face.
- drawDegrees: Draws degree markers on the display.
- drawBorders: Draws border rings on the display.
- draw_scale_ticks: Draws scale ticks on the clock face.
- drawPie: Draws a pie shape on the display.
- drawRoundTimer: Draws a round timer on the display.
- drawRoundDashBoard: Draws a round dashboard on the display.
- drawFaceTHREE: Draws a third round face on the display.
- drawDigitalTime: Displays digital time.
- infoBarGraph: Displays bar graphs for sensor data.
- show_battery_level: Displays the battery level.
- update_time: Updates the time counter.
- clearDisplay: Clears the display.
- convert_to_range: Converts a value to a specified range.
- convert_values_to_range: Converts three values to a specified range.
The main loop continuously updates the display with the current time, sensor data, and graphical elements.

CREDITS:
GFX and Text from Tony Goodhew routines are included in full - enjoy# 15th January 2025
"""

from machine import Pin,I2C,SPI,PWM,ADC # type: ignore
import framebuf # type: ignore
import time
import math

# PIN DEFINITIONS
# SPI for Display
DC = 8
CS = 9
SCK = 10
MOSI = 11
RST = 12
BL = 25
# I2C for qmi8658 Accelerometer and Gyroscope
SCL = 7
SDA = 6
# BATTERY VOLTAGE Measuring Pin
Vbat_Pin = 29

# DISPLAY SPECIFICATIONS
# Dimensions
width = 240
height = 240
# Centre Coordinates
xc = 120  
yc = 120
r = 120 

# Clock Hands Length
mr = 100  # length of minite hand       
hr = 70   # length of hour hand
sr = 14  # length of second hand

# Start time
h = 11
m = 40
s = 59

#CUBE SPECIFICATIONS
# Initial angle
angle_x = 0
angle_y = 0
angle_z = 0
# Scale factor for vertices
scale_factor = 0.008
cube_radius = 0.1
# Cube vertices
cube_vertices = [
    [-24 * scale_factor, -24 * scale_factor, -24 * scale_factor],
    [24 * scale_factor, -24 * scale_factor, -24 * scale_factor],
    [24 * scale_factor, 24 * scale_factor, -24 * scale_factor],
    [-24 * scale_factor, 24 * scale_factor, -24 * scale_factor],
    [-24 * scale_factor, -24 * scale_factor, 24 * scale_factor],
    [24 * scale_factor, -24 * scale_factor, 24 * scale_factor],
    [24 * scale_factor, 24 * scale_factor, 24 * scale_factor],
    [-24 * scale_factor, 24 * scale_factor, 24 * scale_factor]
]
# Cube edges
cube_edges = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7)
]

# CLASS DEFINITIONS ------------------------------------------------------------------

# Waveshare RP2040 1.28" IPS LCD Board Class
class LCD_1inch28(framebuf.FrameBuffer): 
    def __init__(self):
        self.width = 240
        self.height = 240
        
        self.cs = Pin(CS,Pin.OUT)
        self.rst = Pin(RST,Pin.OUT)
        
        self.cs(1)
        self.spi = SPI(1,100_000_000,polarity=0, phase=0,sck=Pin(SCK),mosi=Pin(MOSI),miso=None)
        self.dc = Pin(DC,Pin.OUT)
        self.dc(1)
        self.buffer = bytearray(self.height * self.width * 2)
        super().__init__(self.buffer, self.width, self.height, framebuf.RGB565)
        self.init_display()
        
        self.red   =   0x07E0
        self.green =   0x001f
        self.blue  =   0xf800
        self.white =   0xffff
        
        self.fill(self.white)
        self.show()

        self.pwm = PWM(Pin(BL))
        self.pwm.freq(5000)
        
    def write_cmd(self, cmd):
        self.cs(1)
        self.dc(0)
        self.cs(0)
        self.spi.write(bytearray([cmd]))
        self.cs(1)

    def write_data(self, buf):
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(bytearray([buf]))
        self.cs(1)
    def set_bl_pwm(self,duty):
        self.pwm.duty_u16(duty)#max 65535
    def init_display(self):
        """Initialize display"""  
        self.rst(1)
        time.sleep(0.01)
        self.rst(0)
        time.sleep(0.01)
        self.rst(1)
        time.sleep(0.05)
        
        self.write_cmd(0xEF)
        self.write_cmd(0xEB)
        self.write_data(0x14) 
        
        self.write_cmd(0xFE) 
        self.write_cmd(0xEF) 

        self.write_cmd(0xEB)
        self.write_data(0x14) 

        self.write_cmd(0x84)
        self.write_data(0x40) 

        self.write_cmd(0x85)
        self.write_data(0xFF) 

        self.write_cmd(0x86)
        self.write_data(0xFF) 

        self.write_cmd(0x87)
        self.write_data(0xFF)

        self.write_cmd(0x88)
        self.write_data(0x0A)

        self.write_cmd(0x89)
        self.write_data(0x21) 

        self.write_cmd(0x8A)
        self.write_data(0x00) 

        self.write_cmd(0x8B)
        self.write_data(0x80) 

        self.write_cmd(0x8C)
        self.write_data(0x01) 

        self.write_cmd(0x8D)
        self.write_data(0x01) 

        self.write_cmd(0x8E)
        self.write_data(0xFF) 

        self.write_cmd(0x8F)
        self.write_data(0xFF) 


        self.write_cmd(0xB6)
        self.write_data(0x00)
        self.write_data(0x20)

        self.write_cmd(0x36)
        self.write_data(0x98)

        self.write_cmd(0x3A)
        self.write_data(0x05) 


        self.write_cmd(0x90)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x08) 

        self.write_cmd(0xBD)
        self.write_data(0x06)
        
        self.write_cmd(0xBC)
        self.write_data(0x00)

        self.write_cmd(0xFF)
        self.write_data(0x60)
        self.write_data(0x01)
        self.write_data(0x04)

        self.write_cmd(0xC3)
        self.write_data(0x13)
        self.write_cmd(0xC4)
        self.write_data(0x13)

        self.write_cmd(0xC9)
        self.write_data(0x22)

        self.write_cmd(0xBE)
        self.write_data(0x11) 

        self.write_cmd(0xE1)
        self.write_data(0x10)
        self.write_data(0x0E)

        self.write_cmd(0xDF)
        self.write_data(0x21)
        self.write_data(0x0c)
        self.write_data(0x02)

        self.write_cmd(0xF0)   
        self.write_data(0x45)
        self.write_data(0x09)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x26)
        self.write_data(0x2A)

        self.write_cmd(0xF1)    
        self.write_data(0x43)
        self.write_data(0x70)
        self.write_data(0x72)
        self.write_data(0x36)
        self.write_data(0x37)  
        self.write_data(0x6F)


        self.write_cmd(0xF2)   
        self.write_data(0x45)
        self.write_data(0x09)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x26)
        self.write_data(0x2A)

        self.write_cmd(0xF3)   
        self.write_data(0x43)
        self.write_data(0x70)
        self.write_data(0x72)
        self.write_data(0x36)
        self.write_data(0x37) 
        self.write_data(0x6F)

        self.write_cmd(0xED)
        self.write_data(0x1B) 
        self.write_data(0x0B) 

        self.write_cmd(0xAE)
        self.write_data(0x77)
        
        self.write_cmd(0xCD)
        self.write_data(0x63)


        self.write_cmd(0x70)
        self.write_data(0x07)
        self.write_data(0x07)
        self.write_data(0x04)
        self.write_data(0x0E) 
        self.write_data(0x0F) 
        self.write_data(0x09)
        self.write_data(0x07)
        self.write_data(0x08)
        self.write_data(0x03)

        self.write_cmd(0xE8)
        self.write_data(0x34)

        self.write_cmd(0x62)
        self.write_data(0x18)
        self.write_data(0x0D)
        self.write_data(0x71)
        self.write_data(0xED)
        self.write_data(0x70) 
        self.write_data(0x70)
        self.write_data(0x18)
        self.write_data(0x0F)
        self.write_data(0x71)
        self.write_data(0xEF)
        self.write_data(0x70) 
        self.write_data(0x70)

        self.write_cmd(0x63)
        self.write_data(0x18)
        self.write_data(0x11)
        self.write_data(0x71)
        self.write_data(0xF1)
        self.write_data(0x70) 
        self.write_data(0x70)
        self.write_data(0x18)
        self.write_data(0x13)
        self.write_data(0x71)
        self.write_data(0xF3)
        self.write_data(0x70) 
        self.write_data(0x70)

        self.write_cmd(0x64)
        self.write_data(0x28)
        self.write_data(0x29)
        self.write_data(0xF1)
        self.write_data(0x01)
        self.write_data(0xF1)
        self.write_data(0x00)
        self.write_data(0x07)

        self.write_cmd(0x66)
        self.write_data(0x3C)
        self.write_data(0x00)
        self.write_data(0xCD)
        self.write_data(0x67)
        self.write_data(0x45)
        self.write_data(0x45)
        self.write_data(0x10)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)

        self.write_cmd(0x67)
        self.write_data(0x00)
        self.write_data(0x3C)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x01)
        self.write_data(0x54)
        self.write_data(0x10)
        self.write_data(0x32)
        self.write_data(0x98)

        self.write_cmd(0x74)
        self.write_data(0x10)
        self.write_data(0x85)
        self.write_data(0x80)
        self.write_data(0x00) 
        self.write_data(0x00) 
        self.write_data(0x4E)
        self.write_data(0x00)
        
        self.write_cmd(0x98)
        self.write_data(0x3e)
        self.write_data(0x07)

        self.write_cmd(0x35)
        self.write_cmd(0x21)

        self.write_cmd(0x11)
        time.sleep(0.12)
        self.write_cmd(0x29)
        time.sleep(0.02)
        
        self.write_cmd(0x21)

        self.write_cmd(0x11)

        self.write_cmd(0x29)

    def show(self):
        self.write_cmd(0x2A)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0xef)
        
        self.write_cmd(0x2B)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0xEF)
        
        self.write_cmd(0x2C)
        
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(self.buffer)
        self.cs(1)

# QMI8658 Accelerometer and Gyroscope Class
class QMI8658(object):
    def __init__(self,address=0X6B):
        self._address = address
        self._bus = I2C(id=1,scl=Pin(SCL),sda=Pin(SDA),freq=100_000)
        bRet=self.WhoAmI()
        if bRet :
            self.Read_Revision()
        else    :
            return None
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
            xyz[i]=raw_xyz[i]/acc_lsb_div
            xyz[i+3]=raw_xyz[i+3]*1.0/gyro_lsb_div
        return xyz

# Cartesian Graph Class
class CartesianGraph:
    def __init__(self, scale, origin_x, origin_y, width=50, height=20):
        self.scale = scale
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        self.values1 = []
        self.values2 = []
        self.values3 = []
        self.max_points = width  # Maximum number of points to display

    def add_values(self, value1, value2, value3):
        self.values1.append(value1)
        self.values2.append(value2)
        self.values3.append(value3)
        if len(self.values1) > self.max_points:
            self.values1.pop(0)
            self.values2.pop(0)
            self.values3.pop(0)

    def draw(self):
        # Clear the graph area
        rect(self.origin_x, self.origin_y, self.origin_x + self.width, self.origin_y + self.height +14, colourConverter(0, 0, 0))

        # Draw the graph border
        rect(self.origin_x-4, self.origin_y-4, self.origin_x + self.width + 4, self.origin_y + self.height + 12, colourConverter(55, 55, 55))

        # Define the colors for the three values
        color1 = colourConverter(255, 0, 0)  # Red
        color2 = colourConverter(0, 255, 0)  # Green
        color3 = colourConverter(0, 0, 255)  # Blue

        # Plot the values on the graph
        for i in range(1, len(self.values1)):
            x1 = self.origin_x + (i - 1)
            y1 = self.origin_y + self.height // 2 - int(self.values1[i - 1] * self.scale)
            x2 = self.origin_x + i
            y2 = self.origin_y + self.height // 2 - int(self.values1[i] * self.scale)
            draw_line(x1, y1, x2, y2, color1)

            y1 = self.origin_y + self.height // 2 - int(self.values2[i - 1] * self.scale)
            y2 = self.origin_y + self.height // 2 - int(self.values2[i] * self.scale)
            draw_line(x1, y1, x2, y2, color2)

            y1 = self.origin_y + self.height // 2 - int(self.values3[i - 1] * self.scale)
            y2 = self.origin_y + self.height // 2 - int(self.values3[i] * self.scale)
            draw_line(x1, y1, x2, y2, color3)

# Cube Class
class Cube:
    def __init__(self, vertices, edges, scale_factor=1):
        self.vertices = [[v[0] * scale_factor, v[1] * scale_factor, v[2] * scale_factor] for v in vertices]
        self.edges = edges
        self.angle_x = 0
        self.angle_y = 0
        self.angle_z = 0
        self.width =   240
        self.height =  260
    def project_vertex(self, vertex, angle_x, angle_y, angle_z):
        """Project a 3D vertex to 2D space with rotation around X, Y, and Z axes"""
        x, y, z = vertex
        rad_x = math.radians(angle_x)
        rad_y = math.radians(angle_y)
        rad_z = math.radians(angle_z)
        
        # Rotate around X-axis
        cos_x = math.cos(rad_x)
        sin_x = math.sin(rad_x)
        y_rot = y * cos_x - z * sin_x
        z_rot = y * sin_x + z * cos_x
        
        # Rotate around Y-axis
        cos_y = math.cos(rad_y)
        sin_y = math.sin(rad_y)
        x_rot = x * cos_y + z_rot * sin_y
        z_rot = -x * sin_y + z_rot * cos_y
        
        # Rotate around Z-axis
        cos_z = math.cos(rad_z)
        sin_z = math.sin(rad_z)
        x_final = x_rot * cos_z - y_rot * sin_z
        y_final = x_rot * sin_z + y_rot * cos_z
        
        # Perspective projection
        fov = 256
        viewer_distance = 3
        factor = fov / (viewer_distance + z_rot)
        x_proj = x_final * factor + self.width // 2
        y_proj = y_final * factor + self.height // 2
        
        return int(x_proj), int(y_proj)
    
    
    def draw(self):
        """Draw the cube on the LCD display with current rotation angles"""
        projected_vertices = [self.project_vertex(v, self.angle_x, self.angle_y, self.angle_z) for v in self.vertices]
        first_line_color = colourConverter(50, 150, 250)  
        second_line_color = colourConverter(250, 150, 150)  
        third_line_color = colourConverter(255, 200, 50)  
        color = colourConverter(0, 200, 40)  # Green color for the rest of the lines
        for i, edge in enumerate(self.edges):
            start, end = edge
            x1, y1 = projected_vertices[start]
            x2, y2 = projected_vertices[end]
            if i == 0:
                draw_line(x1, y1, x2, y2, first_line_color)
            elif i == 1:
                draw_line(x1, y1, x2, y2, second_line_color)
            elif i == 9:
                draw_line(x1, y1, x2, y2, third_line_color)
            else:
                draw_line(x1, y1, x2, y2, color)

    def update_angles(self, gyr_x, gyr_y, gyr_z, speed=0.5):
        """Update the rotation angles based on gyroscope values"""
        self.angle_x += gyr_x * speed
        self.angle_y += gyr_y * speed
        self.angle_z += gyr_z * speed

        # Ensure the angles stay within 0-360 degrees
        self.angle_x %= 360
        self.angle_y %= 360
        self.angle_z %= 360

# Point Class
class Point:
    # Draw a Point on the display
    def __init__(self,x,y):
        self.X=x
        self.Y=y
    def __str__(self):
        return "Point(%s,%s)"%(self.X,self.Y) 


# DISPLAY - GRAPHICS and DRAWING FUNCTIONS
# Draw a line
def draw_line(x1, y1, x2, y2, color):
    if x1 == x2:
        # Vertical line
        LCD.vline(x1, min(y1, y2), abs(y2 - y1), color)
    elif y1 == y2:
        # Horizontal line
        LCD.hline(x1, y1, abs(x2 - x1), color)
    else:
        # Diagonal line (Bresenham's line algorithm)
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            LCD.pixel(x1, y1, color)
            if x1 == x2 and y1 == y2:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy

#Draw a Circle (Circle with Fill)
def circle(x,y,r,c):
    LCD.hline(x-r,y,r*2,c)
    for i in range(1,r):
        a = int(math.sqrt(r*r-i*i)) # Pythagoras!
        LCD.hline(x-a,y+i,a*2,c) # Lower half
        LCD.hline(x-a,y-i,a*2,c) # Upper half

#Draw a Ring (Circle without fill)
def ring(x,y,r,c):
    LCD.pixel(x-r,y,c)
    LCD.pixel(x+r,y,c)
    LCD.pixel(x,y-r,c)
    LCD.pixel(x,y+r,c)
    for i in range(1,r):
        a = int(math.sqrt(r*r-i*i))
        LCD.pixel(x-a,y-i,c)
        LCD.pixel(x+a,y-i,c)
        LCD.pixel(x-a,y+i,c)
        LCD.pixel(x+a,y+i,c)
        LCD.pixel(x-i,y-a,c)
        LCD.pixel(x+i,y-a,c)
        LCD.pixel(x-i,y+a,c)
        LCD.pixel(x+i,y+a,c)

#Draw a Rectangle (Without fill)
def rect(x1,y1,x2,y2,c):
    LCD.hline(x1,y1,x2-x1,c)
    LCD.hline(x1,y2,x2-x1,c)
    LCD.vline(x1,y1,y2-y1,c)
    LCD.vline(x2,y1,y2-y1,c)

def character(asc,xt,yt,sz,r,g,b):  # Single character sz is size: 1 or 2
    cc = colourConverter(r,g,b)
    code = asc * 5    # 5 bytes per character
    for ii in range(5):
        line = FONT[code + ii]
        for yy in range(8):
            if (line >> yy) & 0x1:
                LCD.pixel(ii*sz+xt,yy*sz+yt,cc) 
                if sz > 1:
                    LCD.pixel(ii*sz+xt+1,yy*sz+yt,cc)
                    LCD.pixel(ii*sz+xt,yy*sz+yt+1,cc)
                    LCD.pixel(ii*sz+xt+1,yy*sz+yt+1,cc)
                if sz == 3:
                    LCD.pixel(ii*sz+xt,  yy*sz+yt+2,cc)
                    LCD.pixel(ii*sz+xt+1,yy*sz+yt+2,cc)
                    LCD.pixel(ii*sz+xt+2,yy*sz+yt+2,cc)
                    LCD.pixel(ii*sz+xt+2,yy*sz+yt,cc)
                    LCD.pixel(ii*sz+xt+2,yy*sz+yt+1,cc)


    #  FONT SUPPORT BEGIN --------------------------------------------

# Start of FONTS Section
# Standard ASCII 5x8 font
# https://gist.github.com/tdicola/229b3eeddc12d58fb0bc724a9062aa05
FONT_HEIGHT = 8
FONT_WIDTH = 5
FONT = bytes([
    0x00, 0x00, 0x00, 0x00, 0x00, # <space>
    0x3E, 0x5B, 0x4F, 0x5B, 0x3E,
    0x3E, 0x6B, 0x4F, 0x6B, 0x3E,
    0x1C, 0x3E, 0x7C, 0x3E, 0x1C,
    0x18, 0x3C, 0x7E, 0x3C, 0x18,
    0x1C, 0x57, 0x7D, 0x57, 0x1C,
    0x1C, 0x5E, 0x7F, 0x5E, 0x1C,
    0x00, 0x18, 0x3C, 0x18, 0x00,
    0xFF, 0xE7, 0xC3, 0xE7, 0xFF,
    0x00, 0x18, 0x24, 0x18, 0x00,
    0xFF, 0xE7, 0xDB, 0xE7, 0xFF,
    0x30, 0x48, 0x3A, 0x06, 0x0E,
    0x26, 0x29, 0x79, 0x29, 0x26,
    0x40, 0x7F, 0x05, 0x05, 0x07,
    0x40, 0x7F, 0x05, 0x25, 0x3F,
    0x5A, 0x3C, 0xE7, 0x3C, 0x5A,
    0x7F, 0x3E, 0x1C, 0x1C, 0x08,
    0x08, 0x1C, 0x1C, 0x3E, 0x7F,
    0x14, 0x22, 0x7F, 0x22, 0x14,
    0x5F, 0x5F, 0x00, 0x5F, 0x5F,
    0x06, 0x09, 0x7F, 0x01, 0x7F,
    0x00, 0x66, 0x89, 0x95, 0x6A,
    0x60, 0x60, 0x60, 0x60, 0x60,
    0x94, 0xA2, 0xFF, 0xA2, 0x94,
    0x08, 0x04, 0x7E, 0x04, 0x08, # UP
    0x10, 0x20, 0x7E, 0x20, 0x10, # Down
    0x08, 0x08, 0x2A, 0x1C, 0x08, # Right
    0x08, 0x1C, 0x2A, 0x08, 0x08, # Left
    0x1E, 0x10, 0x10, 0x10, 0x10,
    0x0C, 0x1E, 0x0C, 0x1E, 0x0C,
    0x30, 0x38, 0x3E, 0x38, 0x30,
    0x06, 0x0E, 0x3E, 0x0E, 0x06,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x5F, 0x00, 0x00,
    0x00, 0x07, 0x00, 0x07, 0x00,
    0x14, 0x7F, 0x14, 0x7F, 0x14,
    0x24, 0x2A, 0x7F, 0x2A, 0x12,
    0x23, 0x13, 0x08, 0x64, 0x62,
    0x36, 0x49, 0x56, 0x20, 0x50,
    0x00, 0x08, 0x07, 0x03, 0x00,
    0x00, 0x1C, 0x22, 0x41, 0x00,
    0x00, 0x41, 0x22, 0x1C, 0x00,
    0x2A, 0x1C, 0x7F, 0x1C, 0x2A,
    0x08, 0x08, 0x3E, 0x08, 0x08,
    0x00, 0x80, 0x70, 0x30, 0x00,
    0x08, 0x08, 0x08, 0x08, 0x08,
    0x00, 0x00, 0x60, 0x60, 0x00,
    0x20, 0x10, 0x08, 0x04, 0x02,
    0x3E, 0x51, 0x49, 0x45, 0x3E,
    0x00, 0x42, 0x7F, 0x40, 0x00,
    0x72, 0x49, 0x49, 0x49, 0x46,
    0x21, 0x41, 0x49, 0x4D, 0x33,
    0x18, 0x14, 0x12, 0x7F, 0x10,
    0x27, 0x45, 0x45, 0x45, 0x39,
    0x3C, 0x4A, 0x49, 0x49, 0x31,
    0x41, 0x21, 0x11, 0x09, 0x07,
    0x36, 0x49, 0x49, 0x49, 0x36,
    0x46, 0x49, 0x49, 0x29, 0x1E,
    0x00, 0x00, 0x14, 0x00, 0x00,
    0x00, 0x40, 0x34, 0x00, 0x00,
    0x00, 0x08, 0x14, 0x22, 0x41,
    0x14, 0x14, 0x14, 0x14, 0x14,
    0x00, 0x41, 0x22, 0x14, 0x08,
    0x02, 0x01, 0x59, 0x09, 0x06,
    0x3E, 0x41, 0x5D, 0x59, 0x4E,
    0x7C, 0x12, 0x11, 0x12, 0x7C, # A
    0x7F, 0x49, 0x49, 0x49, 0x36,
    0x3E, 0x41, 0x41, 0x41, 0x22,
    0x7F, 0x41, 0x41, 0x41, 0x3E,
    0x7F, 0x49, 0x49, 0x49, 0x41,
    0x7F, 0x09, 0x09, 0x09, 0x01,
    0x3E, 0x41, 0x41, 0x51, 0x73,
    0x7F, 0x08, 0x08, 0x08, 0x7F,
    0x00, 0x41, 0x7F, 0x41, 0x00,
    0x20, 0x40, 0x41, 0x3F, 0x01,
    0x7F, 0x08, 0x14, 0x22, 0x41,
    0x7F, 0x40, 0x40, 0x40, 0x40,
    0x7F, 0x02, 0x1C, 0x02, 0x7F,
    0x7F, 0x04, 0x08, 0x10, 0x7F,
    0x3E, 0x41, 0x41, 0x41, 0x3E,
    0x7F, 0x09, 0x09, 0x09, 0x06,
    0x3E, 0x41, 0x51, 0x21, 0x5E,
    0x7F, 0x09, 0x19, 0x29, 0x46,
    0x26, 0x49, 0x49, 0x49, 0x32,
    0x03, 0x01, 0x7F, 0x01, 0x03,
    0x3F, 0x40, 0x40, 0x40, 0x3F,
    0x1F, 0x20, 0x40, 0x20, 0x1F,
    0x3F, 0x40, 0x38, 0x40, 0x3F,
    0x63, 0x14, 0x08, 0x14, 0x63,
    0x03, 0x04, 0x78, 0x04, 0x03,
    0x61, 0x59, 0x49, 0x4D, 0x43,
    0x00, 0x7F, 0x41, 0x41, 0x41,
    0x02, 0x04, 0x08, 0x10, 0x20,
    0x00, 0x41, 0x41, 0x41, 0x7F,
    0x04, 0x02, 0x01, 0x02, 0x04,
    0x40, 0x40, 0x40, 0x40, 0x40,
    0x00, 0x03, 0x07, 0x08, 0x00,
    0x20, 0x54, 0x54, 0x78, 0x40,
    0x7F, 0x28, 0x44, 0x44, 0x38,
    0x38, 0x44, 0x44, 0x44, 0x28,
    0x38, 0x44, 0x44, 0x28, 0x7F,
    0x38, 0x54, 0x54, 0x54, 0x18,
    0x00, 0x08, 0x7E, 0x09, 0x02,
    0x18, 0xA4, 0xA4, 0x9C, 0x78,
    0x7F, 0x08, 0x04, 0x04, 0x78,
    0x00, 0x44, 0x7D, 0x40, 0x00,
    0x20, 0x40, 0x40, 0x3D, 0x00,
    0x7F, 0x10, 0x28, 0x44, 0x00,
    0x00, 0x41, 0x7F, 0x40, 0x00,
    0x7C, 0x04, 0x78, 0x04, 0x78,
    0x7C, 0x08, 0x04, 0x04, 0x78,
    0x38, 0x44, 0x44, 0x44, 0x38,
    0xFC, 0x18, 0x24, 0x24, 0x18,
    0x18, 0x24, 0x24, 0x18, 0xFC,
    0x7C, 0x08, 0x04, 0x04, 0x08,
    0x48, 0x54, 0x54, 0x54, 0x24,
    0x04, 0x04, 0x3F, 0x44, 0x24,
    0x3C, 0x40, 0x40, 0x20, 0x7C,
    0x1C, 0x20, 0x40, 0x20, 0x1C,
    0x3C, 0x40, 0x30, 0x40, 0x3C,
    0x44, 0x28, 0x10, 0x28, 0x44,
    0x4C, 0x90, 0x90, 0x90, 0x7C,
    0x44, 0x64, 0x54, 0x4C, 0x44,
    0x00, 0x08, 0x36, 0x41, 0x00,
    0x00, 0x00, 0x77, 0x00, 0x00,
    0x00, 0x41, 0x36, 0x08, 0x00,
    0x02, 0x01, 0x02, 0x04, 0x02,
    0x3C, 0x26, 0x23, 0x26, 0x3C,
    0x1E, 0xA1, 0xA1, 0x61, 0x12, # Extension starts here
    0x3A, 0x40, 0x40, 0x20, 0x7A,
    0x38, 0x54, 0x54, 0x55, 0x59,
    0x21, 0x55, 0x55, 0x79, 0x41,
    0x22, 0x54, 0x54, 0x78, 0x42, # a-umlaut
    0x21, 0x55, 0x54, 0x78, 0x40,
    0x20, 0x54, 0x55, 0x79, 0x40,
    0x0C, 0x1E, 0x52, 0x72, 0x12,
    0x39, 0x55, 0x55, 0x55, 0x59,
    0x39, 0x54, 0x54, 0x54, 0x59,
    0x39, 0x55, 0x54, 0x54, 0x58,
    0x00, 0x00, 0x45, 0x7C, 0x41,
    0x00, 0x02, 0x45, 0x7D, 0x42,
    0x00, 0x01, 0x45, 0x7C, 0x40,
    0x7D, 0x12, 0x11, 0x12, 0x7D, # A-umlaut
    0xF0, 0x28, 0x25, 0x28, 0xF0,
    0x7C, 0x54, 0x55, 0x45, 0x00,
    0x20, 0x54, 0x54, 0x7C, 0x54,
    0x7C, 0x0A, 0x09, 0x7F, 0x49,
    0x32, 0x49, 0x49, 0x49, 0x32,
    0x3A, 0x44, 0x44, 0x44, 0x3A, # o-umlaut
    0x32, 0x4A, 0x48, 0x48, 0x30,
    0x3A, 0x41, 0x41, 0x21, 0x7A,
    0x3A, 0x42, 0x40, 0x20, 0x78,
    0x00, 0x9D, 0xA0, 0xA0, 0x7D,
    0x3D, 0x42, 0x42, 0x42, 0x3D, # O-umlaut
    0x3D, 0x40, 0x40, 0x40, 0x3D,
    0x3C, 0x24, 0xFF, 0x24, 0x24,
    0x48, 0x7E, 0x49, 0x43, 0x66,
    0x2B, 0x2F, 0xFC, 0x2F, 0x2B,
    0xFF, 0x09, 0x29, 0xF6, 0x20,
    0xC0, 0x88, 0x7E, 0x09, 0x03,
    0x20, 0x54, 0x54, 0x79, 0x41,
    0x00, 0x00, 0x44, 0x7D, 0x41,
    0x30, 0x48, 0x48, 0x4A, 0x32,
    0x38, 0x40, 0x40, 0x22, 0x7A,
    0x00, 0x7A, 0x0A, 0x0A, 0x72,
    0x7D, 0x0D, 0x19, 0x31, 0x7D,
    0x26, 0x29, 0x29, 0x2F, 0x28,
    0x26, 0x29, 0x29, 0x29, 0x26,
    0x30, 0x48, 0x4D, 0x40, 0x20,
    0x38, 0x08, 0x08, 0x08, 0x08,
    0x08, 0x08, 0x08, 0x08, 0x38,
    0x2F, 0x10, 0xC8, 0xAC, 0xBA,
    0x2F, 0x10, 0x28, 0x34, 0xFA,
    0x00, 0x00, 0x7B, 0x00, 0x00,
    0x08, 0x14, 0x2A, 0x14, 0x22,
    0x22, 0x14, 0x2A, 0x14, 0x08,
    0x55, 0x00, 0x55, 0x00, 0x55, # 176 (25% block) missing in old code
    0xAA, 0x55, 0xAA, 0x55, 0xAA, # 50% block
    0xFF, 0x55, 0xFF, 0x55, 0xFF, # 75% block
    0x00, 0x00, 0x00, 0xFF, 0x00,
    0x10, 0x10, 0x10, 0xFF, 0x00,
    0x14, 0x14, 0x14, 0xFF, 0x00,
    0x10, 0x10, 0xFF, 0x00, 0xFF,
    0x10, 0x10, 0xF0, 0x10, 0xF0,
    0x14, 0x14, 0x14, 0xFC, 0x00,
    0x14, 0x14, 0xF7, 0x00, 0xFF,
    0x00, 0x00, 0xFF, 0x00, 0xFF,
    0x14, 0x14, 0xF4, 0x04, 0xFC,
    0x14, 0x14, 0x17, 0x10, 0x1F,
    0x10, 0x10, 0x1F, 0x10, 0x1F,
    0x14, 0x14, 0x14, 0x1F, 0x00,
    0x10, 0x10, 0x10, 0xF0, 0x00,
    0x00, 0x00, 0x00, 0x1F, 0x10,
    0x10, 0x10, 0x10, 0x1F, 0x10,
    0x10, 0x10, 0x10, 0xF0, 0x10,
    0x00, 0x00, 0x00, 0xFF, 0x10,
    0x10, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0xFF, 0x10,
    0x00, 0x00, 0x00, 0xFF, 0x14,
    0x00, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0x00, 0x1F, 0x10, 0x17,
    0x00, 0x00, 0xFC, 0x04, 0xF4,
    0x14, 0x14, 0x17, 0x10, 0x17,
    0x14, 0x14, 0xF4, 0x04, 0xF4,
    0x00, 0x00, 0xFF, 0x00, 0xF7,
    0x14, 0x14, 0x14, 0x14, 0x14,
    0x14, 0x14, 0xF7, 0x00, 0xF7,
    0x14, 0x14, 0x14, 0x17, 0x14,
    0x10, 0x10, 0x1F, 0x10, 0x1F,
    0x14, 0x14, 0x14, 0xF4, 0x14,
    0x10, 0x10, 0xF0, 0x10, 0xF0,
    0x00, 0x00, 0x1F, 0x10, 0x1F,
    0x00, 0x00, 0x00, 0x1F, 0x14,
    0x00, 0x00, 0x00, 0xFC, 0x14,
    0x00, 0x00, 0xF0, 0x10, 0xF0,
    0x10, 0x10, 0xFF, 0x10, 0xFF,
    0x14, 0x14, 0x14, 0xFF, 0x14,
    0x10, 0x10, 0x10, 0x1F, 0x00,
    0x00, 0x00, 0x00, 0xF0, 0x10,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
    0xFF, 0xFF, 0xFF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xFF, 0xFF,
    0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
    0x38, 0x44, 0x44, 0x38, 0x44, # alpha - Greek characters start here  at 224
    0xFC, 0x4A, 0x4A, 0x4A, 0x34, # sharp-s or beta
    0x7E, 0x02, 0x02, 0x06, 0x06,
    0x02, 0x7E, 0x02, 0x7E, 0x02, # pi
    0x63, 0x55, 0x49, 0x41, 0x63,
    0x38, 0x44, 0x44, 0x3C, 0x04,
    0x40, 0x7E, 0x20, 0x1E, 0x20, # mu
    0x06, 0x02, 0x7E, 0x02, 0x02,
    0x99, 0xA5, 0xE7, 0xA5, 0x99,
    0x1C, 0x2A, 0x49, 0x2A, 0x1C,
    0x4C, 0x72, 0x01, 0x72, 0x4C, # omega
    0x30, 0x4A, 0x4D, 0x4D, 0x30,
    0x30, 0x48, 0x78, 0x48, 0x30,
    0xBC, 0x62, 0x5A, 0x46, 0x3D,
    0x3E, 0x49, 0x49, 0x49, 0x00,
    0x7E, 0x01, 0x01, 0x01, 0x7E, # End of Greek chars
    0x2A, 0x2A, 0x2A, 0x2A, 0x2A, # equivalent to 240
    0x44, 0x44, 0x5F, 0x44, 0x44, # + or -
    0x40, 0x51, 0x4A, 0x44, 0x40, # >=
    0x40, 0x44, 0x4A, 0x51, 0x40, # <=
    0x00, 0x00, 0xFF, 0x01, 0x03, # top of integral
    0xE0, 0x80, 0xFF, 0x00, 0x00, # bottom of integral
    0x08, 0x08, 0x6B, 0x6B, 0x08,
    0x36, 0x12, 0x36, 0x24, 0x36, # approximately
    0x06, 0x0F, 0x09, 0x0F, 0x06, # Degree
    0x00, 0x00, 0x18, 0x18, 0x00,
    0x00, 0x00, 0x10, 0x10, 0x00,
    0x30, 0x40, 0xFF, 0x01, 0x01, # sq root
    0x00, 0x1F, 0x01, 0x01, 0x1E, # n superscript
    0x00, 0x19, 0x1D, 0x17, 0x12, # squared (^2)
    0x00, 0x3C, 0x3C, 0x3C, 0x3C,
    0x00, 0x00, 0x00, 0x00, 0x00  # 255 also a <space>
])
      
# Print Text as String
def print_st(asci,xx,yy,sz,r,g,b):  
    if sz == 1: move = 6
    if sz == 2: move = 11
    if sz == 3: move = 17 
    for letter in(asci):
        asci = ord(letter)
        character(asci,xx,yy,sz,r,g,b)
        xx = xx + move

# Centres text on line y and print
def print_yc(s,y,sz,r,g,b): 
    if sz == 1: w = 6
    if sz == 2: w = 11
    if sz == 3: w = 17 
    gap = int((width - len(s) * w)/2)
    print_st(s,gap,y,sz,r,g,b)


# FONT SUPPORT END 


# UTILITY FUNCTIONS --------------------------------------------------------

# Board now setup  MAIN BELOW
def end_point(theta, rr): # Calculate end of hand offsets
    theta_rad = math.radians(theta)    
    xx = int(rr * math.sin(theta_rad))
    yy = -int(rr * math.cos(theta_rad))                     
    return xx,yy

# Convert RGB888 to RGB565
def colourConverter(R,G,B): 
    return (((G&0b00011100)<<3) +((B&0b11111000)>>3)<<8) + (R&0b11111000)+((G&0b11100000)>>5)

# Cnvert value to a range
def convert_to_range(value, min_value, max_value, min_range, max_range):
    """
    Convert a value to a range between -90 and 90.

    :param value: The float value to convert.
    :param min_value: The minimum value of the original range.
    :param max_value: The maximum value of the original range.
    :return: The converted value in the range -90 to 90.
    """
    if min_value >= max_value:
        raise ValueError("min_value must be less than max_value")
    if not (min_value <= value <= max_value):
        raise ValueError("value must be between min_value and max_value")
    
    # Normalize the value to a range between 0 and 1
    normalized_value = (value - min_value) / (max_value - min_value) #    
    # Scale the normalized value to the range -90 to 90
    scaled_value = normalized_value * min_range - max_range
    scaled_value = normalized_value * (max_range - min_range) + min_range
    return scaled_value

# Convert three values to a range of values
def convert_values_to_range(value1, value2, value3, min_value, max_value, min_range, max_range):
    """
    Convert three float values to a range between -90 and 90 and return them as integers.

    :param value1: The first float value to convert.
    :param value2: The second float value to convert.
    :param value3: The third float value to convert.
    :param min_value: The minimum value of the original range.
    :param max_value: The maximum value of the original range.
    :return: A tuple of three converted values as integers.
    """
    converted_value1 = int(convert_to_range(value1, min_value, max_value, min_range, max_range))
    converted_value2 = int(convert_to_range(value2, min_value, max_value, min_range, max_range))
    converted_value3 = int(convert_to_range(value3, min_value, max_value, min_range, max_range))
    
    return converted_value1, converted_value2, converted_value3

# Update timer counter
def update_time():
    global s, m, h
    s += 1
    if s == 60:
        s = 0
        m += 1
        if m == 60:
            m = 0
            h += 1
            if h == 12:
                h = 0

# Round vertex corners
def round_corners(vertices, radius):
    """Round the corners of vertices by a given radius."""
    return [[v * (1 - radius) for v in vertex] for vertex in vertices]


# DRAWING FUNCTIONS ---------------------------------------------------------------

#Clear the dislay
def clearDisplay(c):
    LCD.fill(c)

def draw_line(x1, y1, x2, y2, color):
    """Draw a line using hline and vline"""
    if x1 == x2:
        # Vertical line
        LCD.vline(x1, min(y1, y2), abs(y2 - y1), color)
    elif y1 == y2:
        # Horizontal line
        LCD.hline(x1, y1, abs(x2 - x1), color)
    else:
        # Diagonal line (Bresenham's line algorithm)
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            LCD.pixel(x1, y1, color)
            if x1 == x2 and y1 == y2:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
     

# SCREEN DISPLAY WIDGETS --------------------------------------------------------
# draw logo 
def drawLogo(origin_x=0, origin_y=0, scale=.8):
    coordinates = [
        (0, 0, 26, 22),
        (9, 10, 13, 22),
        (31, 12, 41, 30),
        (45, 12, 55, 24),
        (59, 18, 69, 24),
        (73, 12, 83, 24),
        (87, 12, 109, 24),
        (101, 14, 105, 18)
    ]
    color = colourConverter(255, 255, 255)
    for x1, y1, x2, y2 in coordinates:
        rect(int(x1 * scale) + origin_x, int(y1 * scale) + origin_y, int(x2 * scale) + origin_x, int(y2 * scale) + origin_y, color)

# draw horizontal Bargraph
def drawHorizontalBargraph(min_value, max_value, current_value, graph_color, border_color, bar_x = 40, bar_y = 90):

    """
    Draw a horizontal bargraph on the LCD display screen.
    
    Parameters:
    min_value (int): The minimum value of the bargraph.
    max_value (int): The maximum value of the bargraph.
    current_value (int): The current value to be displayed on the bargraph.
    graph_color (tuple): The color of the bargraph (R, G, B).
    border_color (tuple): The color of the rectangle border (R, G, B).

    Example usage
    draw_horizontal_bargraph(0, 100, 75, colourConverter(0, 255, 0), colourConverter(255, 255, 255))

    """
    # Define the bargraph dimensions and position
    bar_width = 20
    bar_height = 3
    
    # Calculate the width of the filled part of the bargraph
    filled_width = int((current_value - min_value) / (max_value - min_value) * bar_width)
    
    # Draw the border of the bargraph
    rect(bar_x, bar_y, bar_x + bar_width, bar_y + bar_height, border_color)
    
    # Draw the filled part of the bargraph
    rect(bar_x, bar_y, bar_x + filled_width, bar_y + bar_height, graph_color)

# Draw Clock Numbers 1 to 12 in a circle
def drawRoundClockFaceNumbers():
    print_st("7",70,186,1,250,255,250)
    print_st("5",158,186,1,250,255,250)
    print_st("8",42,160,1,250,255,250)
    print_st("4",190,160,1,250,255,250)
    print_st("10",40,70,1,250,255,250)
    print_st("2",190,70,1,250,255,250)
    print_st("11",70,35,1,250,255,250)
    print_st("1",157,35,1,250,255,250)

    print_yc("12",15,2,0,60,100)
    print_yc("6",210,2,0,60,100)
    print_st("9",17,115,2,0,60,100)
    print_st("3",204,115,2,0,60,100)
    
# Draw Degrees 0, 90, 180, 270 on the clock face
def drawDegrees(): #TODO: rename 
    print_yc("0",15,1,30,160,180)
    print_yc("180",220,1,30,160,180)
    print_st("270",17,115,1,30,160,180)
    print_st("90",216,115,1,30,160,180)
    
# Draw the clock face border and ring circles
def drawBorders():
    circle(xc,yc,118,colourConverter(100,120,180))
    ring(xc,yc,114,colourConverter(20,80,255))
    ring(xc,yc,115,colourConverter(250,80,120))
    ring(xc,yc,116,colourConverter(250,80,120))
    circle(xc,yc,114,colourConverter(100,180,120))

# Draw the scale ticks -  lines from centre   
def draw_scale_ticks():
    for p in range(0, 360, 30):
        hxn, hyn = end_point(p, r)
        LCD.line(120, 120, 120 + hxn, 120 + hyn, colourConverter(255, 255, 255))

    #Read XYZ Values form APDS

# Draw an arc/circle segment
def drawPie(xCenterPoint, yCenterPoint, faceRadius, start_angle, end_angle, color):
    """Draw a 3/4 circle (pie) with the cut-out facing down."""
    start_angle = start_angle  # Start angle in degrees
    end_angle = end_angle   # End angle in degrees

    for angle in range(start_angle, end_angle):
        theta_rad = math.radians(angle)
        x = int(xCenterPoint + faceRadius * math.cos(theta_rad))
        y = int(yCenterPoint + faceRadius * math.sin(theta_rad))
        LCD.line(xCenterPoint, yCenterPoint, x, y, color)

# Draw Round Timer
def drawRoundTimer(xCenterPoint = 70, yCenterPoint = 175, faceRadius = 24):
    circle(xCenterPoint, yCenterPoint, faceRadius, colourConverter(150,100,150))
    circle(xCenterPoint, yCenterPoint, faceRadius - 2,colourConverter(30,130,30))
    circle(xCenterPoint, yCenterPoint, faceRadius -4,colourConverter(30,60,10))	#darkline
    circle(xCenterPoint, yCenterPoint, faceRadius -6,colourConverter(180,40,80))
    ring(xCenterPoint, yCenterPoint, faceRadius - 8,colourConverter(250,70,170))
    c = colourConverter(255,255,0)
    LCD.line(xCenterPoint,yCenterPoint,xCenterPoint+sxn,yCenterPoint+syn,c)

    # Draw Round Face TWO

# Draw Round Dashboard
def drawRoundDashBoard(xCenterPoint = xc, yCenterPoint = 190,faceRadius = 24):
    circle(xCenterPoint, yCenterPoint, faceRadius, colourConverter(100,150,150))
    circle(xCenterPoint, yCenterPoint, faceRadius - 2, colourConverter(130,90,20))
    circle(xCenterPoint, yCenterPoint, faceRadius - 4, colourConverter(30,30,50))	#darkline
    circle(xCenterPoint, yCenterPoint, faceRadius - 6, colourConverter(240,180,10))
    ring(xCenterPoint,yCenterPoint, faceRadius - 8, colourConverter(200,250,80))
    drawPie(xCenterPoint, yCenterPoint, faceRadius - 8, -220, 20,colourConverter(20, 20, 20))
    drawPie(xCenterPoint, yCenterPoint, faceRadius - 10, -200, 20,colourConverter(200, 200, 200))
    c = colourConverter(0,0,0)
    LCD.line(xCenterPoint, yCenterPoint, xCenterPoint + mxn, yCenterPoint + myn,c)
    circle(xCenterPoint, yCenterPoint, faceRadius - 20, colourConverter(20,10,10))
    circle(xCenterPoint, yCenterPoint, faceRadius - 22, colourConverter(200,100,10))
    
    # Draw Round Face THREE                                                        

#Draw Round Face THREE
def drawFaceTHREE(xCenterPoint = 170, yCenterPoint = 175, faceRadius = 24):
    circle(xCenterPoint, yCenterPoint, faceRadius, colourConverter(150,100,150))
    circle(xCenterPoint, yCenterPoint, faceRadius - 2, colourConverter(120,30,130))
    circle(xCenterPoint, yCenterPoint, faceRadius - 4, colourConverter(30,60,10))   #darkline
    circle(xCenterPoint, yCenterPoint, faceRadius - 6, colourConverter(10,120,120))

    ring(xCenterPoint, yCenterPoint, faceRadius - 8, colourConverter(200,80,250))
    c = colourConverter(255,255,255)
    LCD.line(xCenterPoint, yCenterPoint, xCenterPoint + hxn, yCenterPoint + hyn,c)

    # Draw Digital Time

# Draw Digital time Face
def drawDigitalTime(xOrigin, yOrigin):
    hs = "0"+str(h)
    ms = "0"+str(m)
    ts = hs[-2:] +":"+ms[-2:]
    print_st(ts,xOrigin,yOrigin,1,0,255,0)
    
# Draw Bargraphs for six values placed in two rows
def infoBarGraph(value1, value2, value3, value4, value5, value6, xOrigin = 40, yOrigin = 70):
    #draw x,y,z allocation text for gyro/accelerometer
    print_st("X",xOrigin + 24,yOrigin - 2,1,200,200,200)
    print_st("Y",xOrigin + 24,yOrigin + 8,1,200,200,200)
    print_st("Z",xOrigin + 24,yOrigin + 18,1,200,200,200)
    #draw Accelerometer bargraph
    drawHorizontalBargraph(-2, 2, value1, colourConverter(255, 255, 100), colourConverter(155, 155, 155), xOrigin,yOrigin)
    drawHorizontalBargraph(-2, 2, value2, colourConverter(0, 100, 255), colourConverter(155, 155, 155), xOrigin,yOrigin + 10)
    drawHorizontalBargraph(-2, 2, value3, colourConverter(255, 10, 10), colourConverter(155, 155, 155), xOrigin,yOrigin + 20)
    #draw Gyroscope bargraph
    drawHorizontalBargraph(-3, 300, value4, colourConverter(100, 200, 255), colourConverter(155, 155, 155), xOrigin + 32,yOrigin)
    drawHorizontalBargraph(-3, 300, value5, colourConverter(50, 255, 50), colourConverter(155, 155, 155), xOrigin + 32,yOrigin + 10)
    drawHorizontalBargraph(-3, 300, value6, colourConverter(255, 0, 255), colourConverter(155, 155, 155), xOrigin + 32,yOrigin + 20)

# Display Battery Status and Level
def show_battery_level(xOrigin = 180, yOrigin = 114):
    vBat = convert_to_range(Vbat.read_u16(), 29000, 43000, 0, 100)
    if 80 <= vBat <= 100:
        color = colourConverter(0, 200, 50)  # Green
    elif 50 <= vBat < 80:
        color = colourConverter(200, 200, 50)  # Yellow
    elif 30 <= vBat < 50:
        color = colourConverter(255, 165, 50)  # Orange
    elif vBat < 30:
        color = colourConverter(255, 0, 0)  # Red
    # Display battery percentage string and %
    print_st("{:.0f}".format(vBat), xOrigin, yOrigin, 1, 205,205,205)  # Message
    print_st("%", xOrigin + 14, yOrigin, 1, 205, 205, 205)  # Message
    # Show Battery Level
    drawHorizontalBargraph(0, 100, int(vBat), color, colourConverter(155, 155, 155), xOrigin, yOrigin + 10)

# END OF DRAW FUNCTIONS ---------------------------------------------------------------


# MAIN PROGRAM STARTS HERE ---------------------------------------------------

# CREATE OBJECTS

# Create LCD object
LCD = LCD_1inch28()            #Initialise the display 
LCD.set_bl_pwm(45535)          # Brightness Original Value 65535

# Create QMI8658 object
qmi8658 = QMI8658()  # Initialise gyro accl
if qmi8658 is None:
    raise RuntimeError("Failed to initialize QMI8658 sensor")

# Create graph object
graph = CartesianGraph(scale=0.2, origin_x=150, origin_y=70)

# Create a Cube object
cube = Cube(cube_vertices, cube_edges)

#  Main loop 

while True:
    
    # Clear the screen
    clearDisplay(0)  

    # Call the function to draw scale ticks
    draw_scale_ticks()

    #Draw Round border lines
    drawBorders()
    
    # Calculate coordinates of end of second pointer
    alpha = (s*6)
    sxn, syn = end_point(alpha, sr)
    # Calculate coordinates of end of minute pointer
    alpha = m * 360/60
    mxn, myn = end_point(alpha, sr)
    # Calculate coordinates of end of hour pointer
    alpha = (h*60+m)*360/720
    hxn, hyn = end_point(alpha, sr)

    # UPDATE ACCELEROMETER AND GYROSCOPE VALUES
    Ax, Ay, Az, Gx, Gy, Gz = qmi8658.Read_XYZ()    # Update Accelerometer and Gyroscope values
    converted_gyro_values = convert_values_to_range(Gx, Gy, Gz, -600, 600, 0, 10)    # Convert Accelerometer values to range
    
    # Start/Update timer
    update_time()
                
    # Add Central circle
    circle(xc,yc,110,colourConverter(30,30,30))
    
    # Add Rotary Degrees 
    drawDegrees()
    
    # Add/Update battery level
    Vbat= ADC(Pin(Vbat_Pin))    #Update battery voltage
    show_battery_level()
    
    # Add Digital clockFace
    drawDigitalTime(136,30)

    # Add Round ClockFace ONE
    drawRoundTimer()
    # Add Round Dashboard
    drawRoundDashBoard(xc, 190, 26)
    # Add Round ClockFace THREE
    drawFaceTHREE()
    
    # Add bargraph for Accelerometer and Gyroscope
    infoBarGraph(Ax, Ay, Az, converted_gyro_values[0],converted_gyro_values[1],converted_gyro_values[2])
    
    # Add Cartesian Graph
    graph.add_values(Gx, Gy, Gz)    # Add new values to the graph
    graph.draw() # Draw the graph
    
    # Update cube angles / Add cube
    cube.update_angles(Gx, Gy, Gz)  # Update the angles of the cube
    cube.draw() # Draw the cube
    
    # Draw logo
    drawLogo(80,36) # Draw the logo at the specified coordinates
    
    # Update screen
    LCD.show()

