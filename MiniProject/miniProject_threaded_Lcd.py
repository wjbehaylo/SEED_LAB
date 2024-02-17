# Mini Project:
# I2C Communication and LCD Trace Library

from smbus2 import SMBus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue


#this function will use threading to output onto the LCD based on queue contents
#value is an integer representing the binaray numbers 00 01 10 11 as integers
def LCD_Display():
    
    #initializing the LCD and the LCD connections to the pi
    lcd_columns = 16
    lcd_rows = 2
    i2c_lcd = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)
    lcd.clear()
    lcd.color = [50, 0, 50]

    #this should be constantly running simultaneously as the main body
    while(True):
        if (not Q.empty()):
            quadrant_int = Q.get()
            quadrant_string = str(quadrant_int//2) + " " + str(quadrant_int%2)
            output = f"I got: {quadrant_string}" 
            print(output) #I may comment this out if it slows down program
            lcd.clear()
            lcd.message = output

#I2C address of arduino, set in Arduino Sketch as well
ARD_ADDR = 8
i2c_arduiino = SMBus(1)

Q = queue.Queue()
Q.put(3)

threadLCD = threading.Thread(target = LCD_Display, args = ())
threadLCD.start()
time.sleep(5)
Q.put(2)
Q.put(2)
time.sleep(5)
Q.put(1)
Q.put(3)
time.sleep(5)
Q.put(0)



