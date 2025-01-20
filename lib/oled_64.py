from machine import Pin, I2C # type: ignore
import ssd1306

SDA = 6
SCL = 7
        
class Oled_64:
    def __init__(self, width, height, sda_pin, scl_pin, freq=400000):
        self.width = width
        self.height = height
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.freq = freq
        self.i2c = self._init_i2c()
        self.display = ssd1306.SSD1306_I2C(width, height, self.i2c)

    def _init_i2c(self):
        sda = Pin(self.sda_pin)
        scl = Pin(self.scl_pin)
        return I2C(1, sda=sda, scl=scl, freq=self.freq)

    def clear_display(self):
        self.display.rotate(False)
        self.display.fill(0)
        self.display.show()

    def drawLogo(self, x_span, y_span):
        # Draw shapes
        self.display.rect(x_span + 62, y_span + 14, 12, 6, 1)  # .
        self.display.rect(x_span + 48, y_span + 8, 12, 14, 1)  # i
        self.display.rect(x_span + 76, y_span + 8, 12, 14, 1)  # i
        self.display.rect(x_span + 34, y_span + 8, 12, 24, 1)  # j
        self.display.rect(x_span + 9, y_span, 24, 22, 1)  # n
        self.display.rect(x_span + 16, y_span + 8, 4, 14, 1)  # n (part 2)
        self.display.rect(x_span + 90, y_span, 24, 22, 1)  # O
        self.display.rect(x_span + 102, y_span + 6, 6, 6, 1)  # O (part 2)

    def display_text(self, text, x, y):
        self.display.text(text, x, y, 1)


# using default address 0x3C

