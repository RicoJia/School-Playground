///*******************************************************************************
//  File Name:
//    main.c
//
//  Summary:
//    SPI interface to control MPC4209 to control output wave 


#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include"spi.h"
#include <math.h>

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

/// \brief Assume we the gain of the DAC is 1, reference voltage is 3.3, this function returns the integer command sent to the DAC to achieve a desired voltage. 
/// \param v_out - desired output voltage. 
/// \return integer command
unsigned short get_DAC_command(float v_out, unsigned char channel){
    
    float v_ref = 3.301;         //reason for this is duty cycle can only be 12 bits... therefore, to achieve 3.3v, you need 12 1s. 
    unsigned short duty_cycle = (int)((0b1000000000000) * v_out / v_ref);
    
     //write to channel A = 0, B = 1; 
    
    unsigned short ret_command = channel << 15 | (0b111 << 12) | duty_cycle; 
    return ret_command; 
}



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
    TRISBbits.TRISB7 = 1;
    
    initSPI();  //initialize SPI
    __builtin_enable_interrupts();
    
    short frequency = 100; 
    short sine_wave_frequency = 2;     
    unsigned char channel_sine = 0;           //channel A on MCP
    const float PI = 3.1415926; 
    float vout_sine = 0.0; 

    
    short int square_wave_frequency = 1; 
    unsigned char channel_square = 1;         //channel A on MCP
    float vout_square = 0.0; 
    
    int count = 0; 
    const float max_voltage = 3.3/2.0;        //voltage peak
    const float y_offset = 3.3/2.0;         //because the MCP can only output 0v - 3.3v, we need an offset to shift the waves to above 0v. 

    while (1) {
        
        LATBINV = 0x0020;           // invert LATB5 value for a heart beat LED (note that this is hex number, 20 is actually 32 in decimal)
        
        // sine wave function that ranges [0,3.3]
        float time = count * 1.0 / frequency; 
        
        vout_sine = max_voltage * sin(2.0* PI * sine_wave_frequency * time) + y_offset; 
        unsigned short sine_command = get_DAC_command(vout_sine, channel_sine); 
        
        spi_io((sine_command>>8));   //send the higher 8 bits
        spi_io(sine_command);        // the lower 8 bits, see if micro controller returns 1 v

        
        LATAbits.LATA0 = 1;     //Bring CS high to finish the first sending cycle
        LATAbits.LATA0 = 0;     //Bring CS low to start the second sending cycle

        // square wave function
        vout_square = vout_square + ((time < 0.5 * (1.0/square_wave_frequency) )? (1.0): (-1.0)) * 4.0*
                                            max_voltage  * square_wave_frequency / frequency; 
        unsigned short square_command = get_DAC_command(vout_square, channel_square); 
        
        spi_io((square_command>>8));   //send the higher 8 bits, which is truncated at the lower 8 bits
        spi_io(square_command);   
        
        // when one period has passed
        ++count;  
        if (count == frequency){
            count = 0; 
        }

        LATAbits.LATA0 = 1;     //Bring CS high
        
        while (_CP0_GET_COUNT()<(48000000/2)/frequency){}   //1hz
            _CP0_SET_COUNT(0);

    }
}