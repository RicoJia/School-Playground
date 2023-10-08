///*******************************************************************************
//  File Name:
//    main.c
//
//  Summary:
//    I2C interface to control MCP 23017 to control output wave 
//    I2C interface to send characters to OLED screen  


#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ws2812b.h"

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
    
    ws2812b_setup(); 
    
    __builtin_enable_interrupts();
    
    
    wsHSB LED0 = {0.0, 1.0, 0.2};
    wsHSB LED1 = {90.0, 1.0, 0.2}; 
    wsHSB LED2 = {180.0, 1.0, 0.2};
    wsHSB LED3 = {270.0, 1.0, 0.2}; 
    int frequency = 20; 

    while(1){
        LATBINV = 0x0020;           // invert LATB5 value for a heart beat LED (note that this is hex number, 20 is actually 32 in decimal)

        wsColor Color0 = HSBtoRGB(LED0.h, LED0.s, LED0.b); 
        wsColor Color1 = HSBtoRGB(LED1.h, LED1.s, LED1.b);         
        wsColor Color2 = HSBtoRGB(LED2.h, LED2.s, LED2.b); 
        wsColor Color3 = HSBtoRGB(LED3.h, LED3.s, LED3.b); 

        wsColor c[] = {Color0, Color1, Color2, Color3}; 
        int num_LED = 4; 
        
        
        ws2812b_setColor(c, num_LED); 
        
        increment_hue(&LED0);        
        increment_hue(&LED1);
        increment_hue(&LED2);
        increment_hue(&LED3);
        
                //        i2c_master_
        while (_CP0_GET_COUNT()<(48000000/2)/frequency){}   //1hz
            _CP0_SET_COUNT(0);
    }
 
    return 0;
}