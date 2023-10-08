///*******************************************************************************
//  File Name:
//    main.c
//
//  Summary:
//    I2C interface to control MCP 23017 to control output wave 
//    I2C interface to send characters to OLED screen  


#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"i2c.h"
#include"imu.h"
#include "ssd1306.h"

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1048576 // use largest wdt (the larger the ratio the better!))
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal 
// QUESTION, FROM 8MHZ -> 48 MHZ??
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations



int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    //initializes B7 as input and B5 as output (0) that is initially off.
    TRISBbits.TRISB5 = 0;
    LATBbits.LATB5 = 0; 

    i2c1_master_setup();               // start setup i2c; 
    i2c2_master_setup();               // start setup i2c; 

    
    ssd1306_setup();
    imu_setup(); 
    
    __builtin_enable_interrupts();
    
//    const short frequency = 1;      //update frequency
    
  
    while (1) {
        
        LATBINV = 0x0020;           // invert LATB5 value for a heart beat LED (note that this is hex number, 20 is actually 32 in decimal)
        
        int length = 14; 
        signed short data[length]; 
        imu_read(IMU_OUT_TEMP_L, data, length); 
        
        unsigned short int chars_size = (128/6)*(32/4);
        double acc_scale = 2 * 9.81/32767; 
        char msg[chars_size]; 
        
        //how to scale Temperature? 
        sprintf(msg, "acc_x: %f || acc_y: %f || acc_z: %f ||", data[4] * acc_scale, data[5]* acc_scale, data[6] * acc_scale); 
        
//        double a_x, a_y, a_z, w_x, w_y, w_z, T; 
//        a_x = 3.0; a_y = 3.0; a_z = 3.0; w_x = 3.0; w_y = 3.0; w_z = 3.0; T = 2.0; 
//        sprintf(msg, "a_x: %f, a_y: %f, a_z: %f, w_x: %f, w_y: %f, w_z: %f, T%f", 
//                a_x, a_y, a_z, w_x, w_y, w_z, T);                //I almost tripped here. 
        
        
        drawMessage(0, 0, msg); 
        
//        //        i2c_master_
//        while (_CP0_GET_COUNT()<(48000000/2)/frequency){}   //1hz
//            _CP0_SET_COUNT(0);
           
    
    }
    
}