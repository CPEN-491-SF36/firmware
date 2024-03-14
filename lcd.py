from RPLCD import *
from time import sleep
from RPLCD.i2c import CharLCD
import RPi.GPIO as GPIO
lcd = CharLCD('PCF8574', 0x27)

# Define LCD states

START = [0, 1]

# Define GPIO pins for buttons
UP_PIN = 17
DOWN_PIN = 18
SELECT_PIN = 22
BACK_PIN = 23

# Setup GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(UP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(DOWN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SELECT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BACK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Define callback functions for button presses
def up_callback(channel):
    move_cursor("up")

def down_callback(channel):
    move_cursor("down")

def select_callback(channel):
    global select_position
    global select_flag

    select_position = cursor_position
    select_flag = True

def back_callback(channel):
    print("Back button pressed")

# Add event detection for each button
GPIO.add_event_detect(UP_PIN, GPIO.FALLING, callback=up_callback, bouncetime=300)
GPIO.add_event_detect(DOWN_PIN, GPIO.FALLING, callback=down_callback, bouncetime=300)
GPIO.add_event_detect(SELECT_PIN, GPIO.FALLING, callback=select_callback, bouncetime=300)
GPIO.add_event_detect(BACK_PIN, GPIO.FALLING, callback=back_callback, bouncetime=300)

def lcd_setup():
    lcd.cursor_pos(0, 0)
    lcd.write_string("Lab in a Shoe")
    lcd.cursor_pos(1, 1)
    lcd.write_string("Start")
    lcd.cursor_pos(2, 1)
    lcd.write_string("Errors")

def lcd_start_state():
    lcd.cursor_pos(0, 0)
    lcd.write_string("Lab in a Shoe")
    lcd.cursor_pos(1, 0)
    lcd.write_string("Recording")
    lcd.cursor_pos(2, 1)
    lcd.write_string("Stop")
    

cursor_position = [1, 0]

def render_cursor():
    lcd.cursor_pos(cursor_position[0], cursor_position[1])
    lcd.write_string(">")

def move_cursor(direction):

    global cursor_position

    if cursor_position[0] == 0:
        holdUp = True

    else
        holdUp = False

    if cursor_position[0] == 3:
        holdDown == True

    else
        holdDown == False

    if direction == "up" and not holdUp:
        cursor_position = [cursor_position[0] - 1, cursor_position[1]]

    elif direction == "down" and not holdDown:
        cursor_position = [cursor_position[0] + 1, cursor_position[1]]

while True:
    render_cursor()

    if select_flag == True:
        select_flag = False
        if select_position == START:
            lcd_start_state()
            lidar.start()
            gp2.start()
            mpu.start()


    

    