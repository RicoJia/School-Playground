#ifndef IMU_H__
#define IMU_H__

#include<xc.h> // processor SFR definitions

#define IMU_WHOAMI 0x0F
#define IMU_REG_W 0b11010110
    #define IMU_REG_R 0b11010111
#define IMU_CTRL1_XL 0x10
#define IMU_CTRL2_G 0x11
#define IMU_CTRL3_C 0x12
#define IMU_OUT_TEMP_L 0x20

#include "i2c.h"

void imu_setup();
void imu_read(unsigned char, signed short *, int);

// High level funcs
unsigned char read_who_am_i(); 

//initialize a register. address is the register address, c is the setting. 
void initialize_reg(unsigned char address, unsigned char c);



#endif

