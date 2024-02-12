import time
import board
import busio
import adafruit_lidarlite
import os
from datetime import datetime
import smbus

ADDRESS1 = 0xE0 >> 1
ADDRESS2 = 0x80 >> 1
ADDRESS3 = 0xD0 >> 1
DISTANCE_REG = 0x5E
SHIFT = 0x35

bus = smbus.SMBus(1)
shift = getShift(ADDRESS1)

def readDistance(address, shift):
    
    bus.write_byte_data(address, DISTANCE_REG, 1)

    high = bus.read_i2c_block_data(address, 0x00, 1)
    low  = bus.read_i2c_block_data(address, 0x00, 1)
    
    return  (float(high) * 16.0 + low) / 16.0 / (2.0 ** shift)

def getShift (address):

    bus.write_byte_data(address, SHIFT, 1)
    bus.read_i2c_block_data(address, 0x00, 1)
    return bus.read_i2c_block_data(address, 0x00, 1)

# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)

# Default configuration, with only i2c wires
sensor1 = adafruit_lidarlite.LIDARLiteV4LED(i2c_bus=i2c, address=0x5C)
sensor2 = adafruit_lidarlite.LIDARLiteV4LED(i2c_bus=i2c, address=0x4C)

i = 0
while os.path.exists("data%s.txt" % i):
        i += 1

fh = open("data%s.txt" %i, "w")

while True:
    try:
        # We print tuples so you can plot with Mu Plotter
        distance1 = sensor1.distance
        distance2 = sensor2.distance
        gp2distance1 = readDistance(ADDRESS1, shift)
        gp2distance2 = readDistance(ADDRESS2, shift)
        gp2distance3 = readDistance(ADDRESS3, shift)
        print("Distance 1: " + str(distance1) + "Distance 2: " + str(distance2))
        print("GP2-1 Distance: " + str(gp2distance1), "GP2-2 Distance: " + str(gp2distance2) + "GP2-3 Distance: " + str(gp2distance3) + "\n")
    
    except:
        # If we get a reading error, just print it and keep truckin'
        print("Failed Sample")

    fh.write("Time = " +  "Sensor 1 Distance: " + str(distance1) + "Sensor 2 Distance:  " + str(distance2) + "\n")

    

