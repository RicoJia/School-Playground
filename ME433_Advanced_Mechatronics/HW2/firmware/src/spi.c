
#include <proc/p32mx170f256b.h>

// initialize SPI1
void initSPI() {
    // Pin B14 has to be SCK1   QUESTION: we don't need to select this? 
    ANSELA = 0;     // Turn of analog pins
    TRISAbits.TRISA0 = 0;   // Make an output pin for CS
    LATAbits.LATA0 = 1;     // Normally it's high
    RPA1Rbits.RPA1R = 0b0011; // Set SDO1
    SDI1Rbits.SDI1R = 0b0001; // Set SDI1   RPB5
    //...

    // setup SPI1
    SPI1CON = 0; // turn off the spi module and reset it        QUESTION: can I do     SPI1CON = 0x00; ? 
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1000; // 1000 for 24kHz, 1 for 12MHz; // baud rate to 10 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi 
}

unsigned char spi_io(unsigned char o){
    SPI1BUF = o;    //MSB to LSB is sent 
    while (!SPI1STATbits.SPIRBF){
        
    }
    return SPI1BUF; 
}
