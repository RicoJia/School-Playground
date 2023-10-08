#ifndef I2C
#define I2C
// Header file for i2c_master_noint.c
// helps implement use I2C1 as a master without using interrupts

#include <xc.h>

void i2c_master_setup(void); // set up I2C1 as master
void i2c_master_start(void); // send a START signal
void i2c_master_restart(void); // send a RESTART signal
void i2c_master_send(unsigned char byte); // send a byte (either an address or data)
unsigned char i2c_master_recv(void); // receive a byte of data
void i2c_master_ack(int val); // send an ACK (0) or NACK (1)
void i2c_master_stop(void); // send a stop

//---------------------------------------------------------Mid level funcs-------------------------
void send_full_command(unsigned char address, unsigned char val);   // send val to register address in a full I2C sending sequence

unsigned char read_pin(unsigned char GPIO_address);    // read a single value returned by MCP chip from the given address

//---------------------------------------------------------Higher level funcs-------------------------
/// \brief These functions are built for blinking an LED on GPA7 and reading push button (detects 0 value) from GPB0 of the MCP23017 chip
void configure_MCP_IO(void); // set all A pins as inputs and all B pins as outputs. 

void invert_LED(void);   //invert the LED value. The initial value of the LED will be 0. 

unsigned char if_button_pressed (void);   //return 1 if the push button on GPB (return value is 0). Else, return 0. 

#endif