from smbus2 import SMBus
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8
# Initialize SMBus library with I2C bus 1
i2c_arduino = SMBus(1)
lcd_columns = 16
lcd_rows = 2

i2c_lcd = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [50, 0, 50]


# Do in a loop
while(True):
    mode = int(input("Which mode should the PI send in? \nDemonstration 1A (1)\nDemonstration 1B (2)\n"))
    # Get user input for offset
    offset = int(input("Enter an offset (7 to quit): "))
    # Provide an exit key
    if(offset == 7):
        break
    if(mode == 1):
        # Get user input for command
        string = input("Enter a string of 32 characters of less:")
        # Write a byte to the i2c bus
        command = [ord(character) for character in string]
        try:
            #aask arduino to take encoder reading
            i2c_arduino.write_i2c_block_data(ARD_ADDR, offset, command)
        except IOError:
            print("Could not write data to the arduino")
    elif(mode == 2):
        #get user input for command
        command = int(input("Enter an integer between 0 and 100: "))
        while(command > 100 or command <0):
            print("Out of range, enter a new input")
            command = int(input("Enter an integer between 0 and 100: "))
        try:
            #aask arduino to take encoder reading
            i2c_arduino.write_byte_data(ARD_ADDR, offset, command)
            sleep(.1)
            reply=i2c_arduino.read_byte_data(ARD_ADDR, offset)
            print("Received from Arduino:" + str(reply))
            lcd.message = str(reply)
            sleep(10)
        except IOError:
            print("Could not write data to the arduino")
        
            
        
