/***************************************************************
  RasPI GP2Y0E02B sensor code
  Gets range from GP2Y0E02B and prints it to the serial monitor.

***************************************************************/

#include <stdio.h>
#include <pigpio.h>
#include <math.h>

#define ADDRESS       0x80 >> 1  // 7 bit addressing so we shift address right one bit
#define DISTANCE_REG  0x5E
#define SHIFT         0x35

int main() {
                                                                                                                              
    float distance = 0;                // Stores the calculated distance 
    uint8_t high, low = 0;              // High and low byte of distance
    float shift = 0;                   // Value in shift bit register

    // Initialize the pigpio library
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Error initializing pigpio library\n");
        return 1;
    }
    
    // Open the I2C interface
    int i2c_handle = i2cOpen(1, ADDRESS, 0);

    if (i2c_handle < 0) {
        fprintf(stderr, "Error opening I2C interface\n");
        gpioTerminate();
        return 1;
    }

    // Write the register address to the device
    if (i2cWriteByte(i2c_handle, SHIFT) < 0) {
        fprintf(stderr, "Error writing shift register address\n");
        i2cClose(i2c_handle);
        gpioTerminate();
        return 1;
    }
    // Read one byte from the device
    if (i2cReadByte(i2c_handle) >= 0) {
        shift = i2cReadByte(i2c_handle);
        printf("Shift value: 0x%x\n", shift);
    } else {
        fprintf(stderr, "Error reading from I2C device\n");
        i2cClose(i2c_handle);
        gpioTerminate();
        return 1;
    }

    while(1){

        // Write the register address to the device
        if (i2cWriteByte(i2c_handle, DISTANCE_REG) < 0) {
            fprintf(stderr, "Error writing distance register address\n");
            i2cClose(i2c_handle);
            gpioTerminate();
            return 1;
        }
        
        high = i2cReadByte(i2c_handle);
        low  = i2cReadByte(i2c_handle);

        distance = (float)(high * 16.0 + low)/16.0/(float)pow(2.0,shift); 

        printf("Distance: %f\n", distance);
       
    }

    // Close the I2C interface
    i2cClose(i2c_handle);

    // Terminate pigpio library
    gpioTerminate();

    return 0;


}
