# MPU Section of this code has been adapted from https://www.electronicwings.com/raspberry-pi/mpu6050-accelerometergyroscope-interfacing-with-raspberry-pi
# 

import multiprocessing
import smbus
import time
import board
import busio
import adafruit_lidarlite
import os
from time import sleep

start_time = time.time()

ADDRESS1 = 0xE0 >> 1
ADDRESS2 = 0x80 >> 1
ADDRESS3 = 0XD0 >> 1
DISTANCE_REG = 0x5E
SHIFT = 0x35

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

i2c = busio.I2C(board.SCL, board.SDA)

#sensor1 = adafruit_lidarlite.LIDARLiteV4LED(i2c_bus=i2c, address = 0x5C)
#sensor2 = adafruit_lidarlite.LIDARLiteV4LED(i2c_bus=i2c, address = 0x4C)

def readDistance(address, shift):

    high = bus.read_i2c_block_data(address, 0x5E, 1)
    low = bus.read_i2c_block_data(address, 0x5F, 1)

    return (float(high[0]) *16.0 + low[0]) / 16.0 / (2.0 **shift[1])

def getShift(address):

    return bus.read_i2c_block(address, SHIFT, 1)

def createDataFile(name):
    
    i = 0
    while os.path.exists(name + "%s.txt" %i):
        i+=1
    
    return open(name + "%s.txt" %i, "w")

def lidarStore(sensor, lidarNum):

    try:
        lidarTime = time.time() - start_time
        distance = sensor.distance
        print(distance)
        lidarData.write("LIDAR_ID: " + str(lidarNum) + "Distance: " + str(distance) + "Time: " +  str(lidarTime) + "\n")
    except:
        pass

def lidarCollect():
    
    try:
        while True:
            lidarStore(sensor2, 2)

    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected. Exiting...")
    finally:
        lidarData.close()
        gp2Data.close()
        print("Files Closed")

def gp2Store(address, gp2Num):

    gp2Time = time.time() - start_time
    distance = readDistance(address, shift)
    gp2Data.write("GP2_ID: " + str(gp2Num) + "Distance: " + str(distance) + "Time: " + str(gp2Time) + "\n")

def gp2Collect():

    try:
        while True:
            gp2Store(ADDRESS3, 3)
    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected. Exiting...")
    finally:
        lidarData.close()
        gp2Data.close()
        print("Files Closed")


def MPU_Init():
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        
        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        
        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)
        
        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        
        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
        #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

def mpuCollect():
    try:
        while True:
            #Read Accelerometer raw value
            #Read Accelerometer raw value
            acc_x = read_raw_data(ACCEL_XOUT_H)
            acc_y = read_raw_data(ACCEL_YOUT_H)
            acc_z = read_raw_data(ACCEL_ZOUT_H)
        
            #Read Gyroscope raw value
            gyro_x = read_raw_data(GYRO_XOUT_H)
            gyro_y = read_raw_data(GYRO_YOUT_H)
            gyro_z = read_raw_data(GYRO_ZOUT_H)
        
            #Full scale range +/- 250 degree/C as per sensitivity scale factor
            Ax = acc_x/16384.0
            Ay = acc_y/16384.0
            Az = acc_z/16384.0
        
            Gx = gyro_x/131.0
            Gy = gyro_y/131.0
            Gz = gyro_z/131.0
        
            now = time.time()-start_time
            print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)         
            mpuData.write("Time: " + str(now) + " Gx=%.2f" %Gx + u'\u00b0'+ "/s"+ "\tGy=%.2f" %Gy+ u'\u00b0'+ "/s"+ "\tGz=%.2f" %Gz+ u'\u00b0'+ "/s"+ "\tAx=%.2f g" %Ax+ "\tAy=%.2f g" %Ay+ "\tAz=%.2f g" %Az + "\n") 
            sleep(1)

    except KeyboardInterrupt:
        print("Keyboard Interrupt detected. Exiting ...")
    finally:
        mpuData.close()
        print("Files Closed")

# I2C Setup
bus = smbus.SMBus(1)

# MPU Setup
Device_Address = 0x68

# GP2 Setup
#shift = getShift(ADDRESS3)

MPU_Init()

# Create Data Files

lidarData = createDataFile("lidar_")
gp2Data   = createDataFile("gp2_")
mpuData   = createDataFile("mpu_")

# Create Processes for Each Sensor
lidar = multiprocessing.Process(target=lidarCollect)
gp2   = multiprocessing.Process(target=gp2Collect)
mpu   = multiprocessing.Process(target=mpuCollect)

# Start Processes

#lidar.start()
#gp2.start()
mpu.start()


