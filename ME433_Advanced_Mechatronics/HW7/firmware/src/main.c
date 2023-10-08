///*******************************************************************************
//  File Name:
//    main.c
//
//  Summary:
// Capacitive touch device using CTMU


#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "adc.h"
#include "ssd1306.h"
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

    adc_setup();    
    ctmu_setup(); 

    i2c1_master_setup();               // start setup i2c; 
    ssd1306_setup();
    
    //set up the addressable LEDs
    ws2812b_setup(); 
    wsHSB LED0 = {0.0, 1.0, 0.2};
    wsHSB LED1 = {0.0, 1.0, 0.2}; 
    wsHSB LED2 = {0.0, 1.0, 0.2};
    wsHSB LED3 = {0.0, 1.0, 0.2}; 


    __builtin_enable_interrupts();
    
    const int pin1 = 5;   //AN5
    const int delay = 48000000 / 2 / 1000/200; //corresponding to 0.005ms
    
    unsigned short int buffer_size = 5; 
    unsigned short int buffer_ptr_num = 0; 
    int adc_ret_buffer[buffer_size]; 
  
    while (1) {
        LATBINV = 0x0020;           // invert LATB5 value for a heart beat LED (note that this is hex number, 20 is actually 32 in decimal)

        // get average adc value
        int adc_ret = ctmu_read(pin1, delay); 
        buffer_ptr_num= (buffer_ptr_num == buffer_size)?0:buffer_ptr_num+1;
        adc_ret_buffer[buffer_ptr_num] =  adc_ret; 
        int adc_avg = get_average_adc_val(adc_ret_buffer, buffer_size); 
        
        
        unsigned short int chars_size = (128/6)*(32/4);
        char msg[chars_size]; 
        sprintf(msg, "electrode reading : %d", adc_avg); 
        drawMessage(0, 0, msg); 
        
        // change Addressable LED accordingly
        LED0.b =  1.0 * (1.0 - (adc_avg/1023.0));
        LED1.b =  0;
        LED2.b =  0;
        LED3.b =  0;

        wsColor Color0 = HSBtoRGB(LED0.h, LED0.s, LED0.b); 
        wsColor Color1 = HSBtoRGB(LED1.h, LED1.s, LED1.b);         
        wsColor Color2 = HSBtoRGB(LED2.h, LED2.s, LED2.b); 
        wsColor Color3 = HSBtoRGB(LED3.h, LED3.s, LED3.b); 
        wsColor c[] = {Color0, Color1, Color2, Color3}; 
        int num_LED = 4; 
        
        ws2812b_setColor(c, num_LED); 
        
    }
    
}
