#include "imu.h"

void imu_setup(){
    
    unsigned char who = 0;
//    ANSELB = 0;           // if you use port B you should change this
    
    // read from IMU_WHOAMI
    who = read_who_am_i(); 
 
    if (who != 0b01101001){
        while(1){    
        }
    }

    // init IMU_CTRL1_XL, 1.66 kHz, with 2g sensitivity, and 100 Hz filter.
    unsigned char CTRL1_XL_CMD = 0b10000010;
    initialize_reg(IMU_CTRL1_XL, CTRL1_XL_CMD);
    
    // init IMU_CTRL2_G; the sample rate to 1.66 kHz, with 1000 dps sensitivity.
    unsigned char CTRL2_G_CMD = 0b10001000;
    initialize_reg(IMU_CTRL2_G, CTRL2_G_CMD);

    // init IMU_CTRL3_C  Making the IF_INC bit a 1 
    // will enable the ability to read from multiple registers in a row without specifying every register location, saving us time when reading out all of the data.
    unsigned char CTRL3_C_CMD = 0b00000100;
    initialize_reg(IMU_CTRL3_C, CTRL3_C_CMD);

}

void imu_read(unsigned char reg, signed short * data, int len){

    // read multiple from the imu, each data takes 2 reads so you need len*2 chars
    //prep work
    i2c1_master_start();                     //send a start bit
    i2c1_master_send(IMU_REG_W);         //write to the write register in MCP that you want to read from the GPIO address
    i2c1_master_send(reg);
    i2c1_master_restart(); 
    i2c1_master_send(IMU_REG_R);          // start to read from the read register in MCP

    unsigned int i = 0; 
    
    for (; i < len; ++i){
        //Receive low byte and high byte
        unsigned char low_byte = i2c1_master_recv(); //waiting for the returned byte
        i2c1_master_ack(0);   
        unsigned char high_byte = i2c1_master_recv(); //waiting for the returned byte
        // turn the chars into the shorts
        data[i] = ((high_byte)<<8) | low_byte; 
        i2c1_master_ack(0);   
    }
    
    i2c1_master_recv(); 
    i2c1_master_ack(1);     
    i2c1_master_stop(); 
   
}

unsigned char read_who_am_i(){
    i2c1_master_start();                     //send a start bit
    i2c1_master_send(IMU_REG_W);         //write to the write register in MCP that you want to read from the GPIO address
    i2c1_master_send(IMU_WHOAMI);
    i2c1_master_restart(); 
    i2c1_master_send(IMU_REG_R);          // start to read from the read register in MCP
    unsigned char ret_val = i2c1_master_recv(); //waiting for the returned byte
    i2c1_master_ack(1); 
    i2c1_master_stop(); 
    return ret_val; 

}

void initialize_reg(unsigned char address, unsigned char c) {
    i2c1_master_start();
    i2c1_master_send(IMU_REG_W);
    i2c1_master_send(address); 
    i2c1_master_send(c);
    i2c1_master_stop();
}