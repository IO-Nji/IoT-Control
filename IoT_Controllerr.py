# IoT Controller Class

"""
IoT Controllerr - Demo Code
Author: Nji S. from https://nji.io
Date: 22nd June 2022
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
- Battery: Manages the battery level measurement.
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
GFX and Text from Tony Goodhew routines are included in full - enjoy# 22nd June 2022
"""

from machine import Pin,I2C,SPI,PWM,ADC # type: ignore
from battery import Battery
from oled_64 import Oled_64
from qmi8658 import QMI8658
from lcdDisplay import LCD_1inch28
import framebuf # type: ignore
import time
import math

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
        LCD.rect(self.origin_x, self.origin_y, self.origin_x + self.width, self.origin_y + self.height +14, LCD.colourConverter(0, 0, 0))

        # Draw the graph border
        LCD.rect(self.origin_x-4, self.origin_y-4, self.origin_x + self.width + 4, self.origin_y + self.height + 12, LCD.colourConverter(55, 55, 55))

        # Define the colors for the three values
        color1 = LCD.colourConverter(255, 0, 0)  # Red
        color2 = LCD.colourConverter(0, 255, 0)  # Green
        color3 = LCD.colourConverter(0, 0, 255)  # Blue

        # Plot the values on the graph
        for i in range(1, len(self.values1)):
            x1 = self.origin_x + (i - 1)
            y1 = self.origin_y + self.height // 2 - int(self.values1[i - 1] * self.scale)
            x2 = self.origin_x + i
            y2 = self.origin_y + self.height // 2 - int(self.values1[i] * self.scale)
            LCD.drawLine(x1, y1, x2, y2, color1)

            y1 = self.origin_y + self.height // 2 - int(self.values2[i - 1] * self.scale)
            y2 = self.origin_y + self.height // 2 - int(self.values2[i] * self.scale)
            LCD.drawLine(x1, y1, x2, y2, color2)

            y1 = self.origin_y + self.height // 2 - int(self.values3[i - 1] * self.scale)
            y2 = self.origin_y + self.height // 2 - int(self.values3[i] * self.scale)
            LCD.drawLine(x1, y1, x2, y2, color3)
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
        first_line_color = LCD.colourConverter(50, 150, 250)  
        second_line_color = LCD.colourConverter(250, 150, 150)  
        third_line_color = LCD.colourConverter(255, 200, 50)  
        color = LCD.colourConverter(0, 200, 40)  # Green color for the rest of the lines
        for i, edge in enumerate(self.edges):
            start, end = edge
            x1, y1 = projected_vertices[start]
            x2, y2 = projected_vertices[end]
            if i == 0:
                LCD.drawLine(x1, y1, x2, y2, first_line_color)
            elif i == 1:
                LCD.drawLine(x1, y1, x2, y2, second_line_color)
            elif i == 9:
                LCD.drawLine(x1, y1, x2, y2, third_line_color)
            else:
                LCD.drawLine(x1, y1, x2, y2, color)

    def update_angles(self, gyr_x, gyr_y, gyr_z, speed = 1):
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


# UTILITY FUNCTIONS ---------------------------------------------------------------
# Board now setup  MAIN BELOW
def end_point(theta, rr): # Calculate end of hand offsets
    theta_rad = math.radians(theta)    
    xx = int(rr * math.sin(theta_rad))
    yy = -int(rr * math.cos(theta_rad))                     
    return xx,yy
# Cnvert value to a range
def convert_to_range(value, min_value, max_value, min_range, max_range):
    """
    Convert a value to a range between -90 and 90.
a
    :param value: The float value to convert.
    :param min_value: The minimum value of the original range.
    :param max_value: The maximum value of the original range.
    :return: The converted value in the range -90 to 90.
    """
    if min_value >= max_value:
        raise ValueError("min_value must be less than max_value")
    if not (min_value <= value <= max_value):
        value = 0
        # raise ValueError("value must be between min_value and max_value")  
    # Normalize the value to a range between 0 and 1
    normalized_value = (value - min_value) / (max_value - min_value) #    
    # Scale the normalized value to the range -90 to 90
    scaled_value = normalized_value * min_range - max_range
    scaled_value = normalized_value * (max_range - min_range) + min_range
    return scaled_value
# Convert three values to a range of values
def convert_values_to_range(v1, v2, v3, min_value, max_value, min_range, max_range):
    """
    Convert three float values to a range between -90 and 90 and return them as integers.

    :param value1: The first float value to convert.
    :param value2: The second float value to convert.
    :param value3: The third float value to convert.
    :param min_value: The minimum value of the original range.
    :param max_value: The maximum value of the original range.
    :return: A tuple of three converted values as integers.
    """
    v1_c = int(convert_to_range(v1, min_value, max_value, min_range, max_range))
    v2_c = int(convert_to_range(v2, min_value, max_value, min_range, max_range))
    v3_c = int(convert_to_range(v3, min_value, max_value, min_range, max_range))
    
    return v1_c, v2_c, v3_c
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
# Calculate coordinates of end of second pointer
def calcPointerPos():
    alpha = (s*6)
    sxn, syn = end_point(alpha, sr)
    # Calculate coordinates of end of minute pointer
    alpha = m * 360/60
    mxn, myn = end_point(alpha, sr)
    # Calculate coordinates of end of hour pointer
    alpha = (h*60+m)*360/720
    hxn, hyn = end_point(alpha, sr)
    return sxn, syn, mxn, myn, hxn, hyn


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
    color = LCD.colourConverter(255, 255, 255)
    for x1, y1, x2, y2 in coordinates:
        LCD.rect(int(x1 * scale) + origin_x, int(y1 * scale) + origin_y, int(x2 * scale) + origin_x, int(y2 * scale) + origin_y, color)
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
    LCD.rect(bar_x, bar_y, bar_x + bar_width, bar_y + bar_height, border_color)
    
    # Draw the filled part of the bargraph
    LCD.rect(bar_x, bar_y, bar_x + filled_width, bar_y + bar_height, graph_color)
# Draw Clock Numbers 1 to 12 in a LCD.circle
def drawRoundClockFaceNumbers():
    LCD.print_st("7",70,186,1,250,255,250)
    LCD.print_st("5",158,186,1,250,255,250)
    LCD.print_st("8",42,160,1,250,255,250)
    LCD.print_st("4",190,160,1,250,255,250)
    LCD.print_st("10",40,70,1,250,255,250)
    LCD.print_st("2",190,70,1,250,255,250)
    LCD.print_st("11",70,35,1,250,255,250)
    LCD.print_st("1",157,35,1,250,255,250)

    LCD.print_yc("12",15,2,0,60,100)
    LCD.print_yc("6",210,2,0,60,100)
    LCD.print_st("9",17,115,2,0,60,100)
    LCD.print_st("3",204,115,2,0,60,100)  
# Draw Degrees 0, 90, 180, 270 on the clock face
def drawDegrees(): #TODO: rename 
    LCD.print_yc("0",15,1,30,160,180)
    LCD.print_yc("180",220,1,30,160,180)
    LCD.print_st("270",17,115,1,30,160,180)
    LCD.print_st("90",216,115,1,30,160,180)
# Draw the clock face border and ring 
def outerRings():
    LCD.circle(xc,yc,118,LCD.colourConverter(100,120,180))
    LCD.ring(xc,yc,114,LCD.colourConverter(20,80,255))
    LCD.ring(xc,yc,115,LCD.colourConverter(250,80,120))
    LCD.ring(xc,yc,116,LCD.colourConverter(250,80,120))
    LCD.circle(xc,yc,114,LCD.colourConverter(100,180,120))
# Display - Round Clock Scale ticks -  lines from centre   
def drawClockScale():
    for p in range(0, 360, 30):
        hxn, hyn = end_point(p, r)
        LCD.line(120, 120, 120 + hxn, 120 + hyn, LCD.colourConverter(255, 255, 255))
# Display - Round Timer
def drawRoundTimer(xCenterPoint = 70, yCenterPoint = 175, faceRadius = 24):
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius, LCD.colourConverter(150,100,150))
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius - 2,LCD.colourConverter(30,130,30))
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius -4,LCD.colourConverter(30,60,10))	#darkline
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius -6,LCD.colourConverter(180,40,80))
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius -6,LCD.colourConverter(150,140,140))
    LCD.ring(xCenterPoint, yCenterPoint, faceRadius - 8,LCD.colourConverter(250,70,170))
    c = LCD.colourConverter(255,255,0)
    LCD.line(xCenterPoint,yCenterPoint,xCenterPoint+calcPointerPos()[0],yCenterPoint+calcPointerPos()[1],c)
# Display - Round Dashboard
def drawRoundDashBoard(xCenterPoint = xc, yCenterPoint = 190,faceRadius = 24):
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius, LCD.colourConverter(100,150,150))
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius - 2, LCD.colourConverter(130,90,20))
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius - 4, LCD.colourConverter(30,30,50))	#darkline
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius - 6, LCD.colourConverter(240,180,10))
    LCD.ring(xCenterPoint,yCenterPoint, faceRadius - 8, LCD.colourConverter(200,250,80))
    LCD.drawPie(xCenterPoint, yCenterPoint, faceRadius - 8, -220, 20,LCD.colourConverter(20, 20, 20))
    LCD.drawPie(xCenterPoint, yCenterPoint, faceRadius - 10, -200, 20,LCD.colourConverter(200, 200, 200))
    c = LCD.colourConverter(0,0,0)
    LCD.line(xCenterPoint, yCenterPoint, xCenterPoint + calcPointerPos()[2], yCenterPoint + calcPointerPos()[3],c)
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius - 20, LCD.colourConverter(20,10,10))
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius - 22, LCD.colourConverter(200,100,10))                                                    
#Draw Round Face THREE
def drawFaceTHREE(xCenterPoint = 170, yCenterPoint = 175, faceRadius = 24):
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius, LCD.colourConverter(150,100,150))
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius - 2, LCD.colourConverter(120,30,130))
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius - 4, LCD.colourConverter(30,60,10))   #darkline
    LCD.circle(xCenterPoint, yCenterPoint, faceRadius - 6, LCD.colourConverter(10,120,120))

    LCD.ring(xCenterPoint, yCenterPoint, faceRadius - 8, LCD.colourConverter(200,80,250))
    c = LCD.colourConverter(255,255,255)
    LCD.line(xCenterPoint, yCenterPoint, xCenterPoint + calcPointerPos()[4], yCenterPoint + calcPointerPos()[5],c)

    # Draw Digital Time
# Draw Digital time Face
def drawDigitalTime(xOrigin, yOrigin):
    hs = "0"+str(h)
    ms = "0"+str(m)
    ts = hs[-2:] +":"+ms[-2:]
    LCD.print_st(ts,xOrigin,yOrigin,1,0,255,0)
 # Draw Bargraphs for six values placed in two rows
# Draw Bargraphs for six values placed in two rows
def infoBarGraph(value1, value2, value3, value4, value5, value6, xOrigin = 40, yOrigin = 70):
    #draw x,y,z allocation text for gyro/accelerometer
    LCD.print_st("X",xOrigin + 24,yOrigin - 2,1,200,200,200)
    LCD.print_st("Y",xOrigin + 24,yOrigin + 8,1,200,200,200)
    LCD.print_st("Z",xOrigin + 24,yOrigin + 18,1,200,200,200)
    #draw Accelerometer bargraph
    drawHorizontalBargraph(-2, 2, value1, LCD.colourConverter(255, 255, 100), LCD.colourConverter(155, 155, 155), xOrigin,yOrigin)
    drawHorizontalBargraph(-2, 2, value2, LCD.colourConverter(0, 100, 255), LCD.colourConverter(155, 155, 155), xOrigin,yOrigin + 10)
    drawHorizontalBargraph(-2, 2, value3, LCD.colourConverter(255, 10, 10), LCD.colourConverter(155, 155, 155), xOrigin,yOrigin + 20)
    #draw Gyroscope bargraph
    drawHorizontalBargraph(-3, 300, value4, LCD.colourConverter(100, 200, 255), LCD.colourConverter(155, 155, 155), xOrigin + 32,yOrigin)
    drawHorizontalBargraph(-3, 300, value5, LCD.colourConverter(50, 255, 50), LCD.colourConverter(155, 155, 155), xOrigin + 32,yOrigin + 10)
    drawHorizontalBargraph(-3, 300, value6, LCD.colourConverter(255, 0, 255), LCD.colourConverter(155, 155, 155), xOrigin + 32,yOrigin + 20)
# Display Battery Status and Level
def displayBattStatus(battLvl, xOrigin=180, yOrigin=114):
     # Display battery percentage string and %
    LCD.print_st("{:.0f}".format(battLvl), xOrigin, yOrigin, 1, 205, 205, 205)  # Message
    LCD.print_st("%", xOrigin + 14, yOrigin, 1, 205, 205, 205)  # Message
    # Show Battery Level
    drawHorizontalBargraph(0, 100, int(battLvl), LCD.colourConverter(20, 255, 40), LCD.colourConverter(155, 155, 155), xOrigin, yOrigin + 10)
# Display Cube
def showCube(Gx, Gy, Gz):
    gyro = convert_values_to_range(Gx, Gy, Gz, -600, 600, 0, 10)    # Convert Accelerometer values to range
    cube.update_angles(Gx, Gy, Gz)  # Update the angles of the cube
    cube.draw() # Draw the cube 
# UPDATE ACCELEROMETER AND GYROSCOPE VALUES
def getAcclGyroValues():
        Ax, Ay, Az, Gx, Gy, Gz = qmi8658.Read_XYZ()
        rawGyro = (Gx, Gy, Gz)
        rawAccl = (Ax, Ay, Az)
        return rawAccl, rawGyro
# Scale Accelerometer and Gyroscope for Graph
def scaleAcclGyro(rawAccl, rawGyro):
    accl = convert_values_to_range(rawAccl[0], rawAccl[1], rawAccl[2], -600, 600, 0, 10)    # Convert Accelerometer values to range
    gyro = convert_values_to_range(rawGyro[0], rawGyro[1], rawGyro[2], -600, 600, 0, 10)    # Convert Gyroscope values to range
    return accl, gyro    
# Show Graph
def showGraph(Gx, Gy, Gz):
    graph.add_values(Gx, Gy, Gz)    # Add new values to the graph
    graph.draw() # Draw the graph



# MAIN PROGRAM STARTS HERE ---------------------------------------------------

# CREATE OBJECTS

# Create LCD object
LCD = LCD_1inch28()            # Initialise the display 
LCD.set_bl_pwm(45535)          # Brightness Original Value 65535

# Create QMI8658 object
qmi8658 = QMI8658()  # Initialise gyro accl
if qmi8658 is None:
    raise RuntimeError("Failed to initialize QMI8658 sensor") #TODO: Use logging

# Create graph object
graph = CartesianGraph(scale=0.2, origin_x=150, origin_y=70)

# Create a Cube object
cube = Cube(cube_vertices, cube_edges)

# Create Battery Object
battery = Battery()
def battStat():   
        voltage = convert_to_range(battery.readVoltage(), 28000, 43000, 0, 100)
        return voltage

# Usage:
oled = Oled_64(128, 64, 6, 7)

def oledLogo():
    oled.clear_display()
    oled.drawLogo(0, 0)
    oled.display_text(" IoT pico", 40, 25)
    oled.display.show()

#  Main loop 
class main():

    # MAIN PROGRAM STARTS HERE ---------------------------------------------------
    def __init__(self):
        pass    

    def run(self):
            
        while True:

            # Display Logo on OLED
            oledLogo()
            
            # Clear the screen
            LCD.fill(0x000000)

            #Get Accelerometer and Gyroscope values
            rawAccl, rawGyro = getAcclGyroValues()
            accl, gyro = scaleAcclGyro(rawAccl, rawGyro)

            # Call the function to draw scale ticks
            drawClockScale()

            #Draw Round border lines
            outerRings()
            
            # Calculate coordinates for pointers
            calcPointerPos()
            
            # Start/Update timer
            update_time()

            # Display Background LCD.circle
            LCD.circle(xc,yc,110,LCD.colourConverter(30, 30,30 ))
            
            # Display Rotary Degrees 
            drawDegrees()
            
            # Display battery info
            displayBattStatus(battStat())
            
            # Display Digital clockFace
            drawDigitalTime(136,30)

            # Display Round ClockFace ONE
            drawRoundTimer()

            # Display Round Dashboard
            drawRoundDashBoard(xc, 190, 26)

            # Display Round ClockFace THREE
            drawFaceTHREE()
            
            # Display Bargraph
            infoBarGraph( gyro[0], gyro[1], gyro[2], accl[0], accl[1], accl[2])
            
            # Display Cartesian Graph
            showGraph(rawGyro[0], rawGyro[1], rawGyro[2])
            
            # Display Cube
            showCube(gyro[0], gyro[1], gyro[2])
            
            # Display logo
            drawLogo(80,36) # Draw the logo at the specified coordinates
            
            # Update screen
            LCD.show()


if __name__ == '__main__':
    main().run()

