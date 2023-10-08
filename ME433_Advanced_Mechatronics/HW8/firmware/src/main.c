///*******************************************************************************
//  File Name:
//    main.c
//
//  Summary:
// RTCC timer. Changes to previous file: FSOSCEN, pin layout. 


#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ssd1306.h"
#include "i2c.h"
#include "rtcc.h"

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = ON // Enable secondary oscillator, which will disable the heartbeat LED
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
    ssd1306_setup();
    
    //clock initialization
    unsigned long init_time = 0x16450000; 
    unsigned long init_date = 0x20060606; 
    rtcc_setup(init_time, init_date); 

    __builtin_enable_interrupts();
    
  
    while (1) {
        
        LATBINV = 0x0020;           // invert LATB5 value for a heart beat LED (note that this is hex number, 20 is actually 32 in decimal)
        
        rtccTime time = readRTCC(); 
        char wday[11];
        dayOfTheWeek(time.wk, wday);
        
        unsigned short int chars_size = (128/6)*(32/4);
        char msg[chars_size]; 
        sprintf(msg, "Date: 20%d%d/%d%d/%d%d | Time: %d%d:%d%d:%d%d |%s " , 
                time.yr10, time.yr01, time.mn10,time.mn01, time.dy10, time.dy01,
                time.hr10, time.hr01, time.min10, time.min01, time.sec10, time.sec01,
                wday); 
        
        drawMessage(0, 0, msg); 
        
        //2hz update
        while (_CP0_GET_COUNT()<(48000000/2)/2){} 
            _CP0_SET_COUNT(0);
    }
    
}
