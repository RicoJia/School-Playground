#ifndef i2c
#define i2c
// Header file for i2c1_master_noint.c
// helps implement use I2C1 as a master without using interrupts


// I2C1 is reserved for the OLED screen, I2C2 is reserved for IMU

#include <xc.h>

void i2c1_master_setup(void); // set up I2C1 as master
void i2c1_master_start(void); // send a START signal
void i2c1_master_restart(void); // send a RESTART signal
void i2c1_master_send(unsigned char byte); // send a byte (either an address or data)
unsigned char i2c1_master_recv(void); // receive a byte of data
void i2c1_master_ack(int val); // send an ACK (0) or NACK (1)
void i2c1_master_stop(void); // send a stop


void i2c2_master_setup(void); // set up I2C2 as master
void i2c2_master_start(void); // send a START signal
void i2c2_master_restart(void); // send a RESTART signal
void i2c2_master_send(unsigned char byte); // send a byte (either an address or data)
unsigned char i2c2_master_recv(void); // receive a byte of data
void i2c2_master_ack(int val); // send an ACK (0) or NACK (1)
void i2c2_master_stop(void); // send a stop    

#endif