// I2C Master utilities, using polling rather than interrupts
// The functions must be called in the correct order as per the I2C protocol
// I2C pins need pull-up resistors, 2k-10k
#include "i2c.h"

void i2c1_master_setup(void) {
    // using a large BRG to see it on the nScope, make it smaller after verifying that code works
    // look up TPGD in the datasheet
    I2C1BRG = 1000; // I2CBRG = [1/(2*Fsck) - TPGD]*Pblck - 2 (TPGD is the Pulse Gobbler Delay) 100khz
    I2C1CONbits.ON = 1; // turn on the I2C1 module
    
    
}

void i2c1_master_start(void) {
    I2C1CONbits.SEN = 1; // send the start bit
    while (I2C1CONbits.SEN) {
        ;
    } // wait for the start bit to be sent
}

void i2c1_master_restart(void) {
    I2C1CONbits.RSEN = 1; // send a restart 
    while (I2C1CONbits.RSEN) {
        ;
    } // wait for the restart to clear
}

void i2c1_master_send(unsigned char byte) { // send a byte to slave
    I2C1TRN = byte; // if an address, bit 0 = 0 for write, 1 for read
    while (I2C1STATbits.TRSTAT) {
        ;
    } // wait for the transmission to finish
    if (I2C1STATbits.ACKSTAT) { // if this is high, slave has not acknowledged
        // ("I2C1 Master: failed to receive ACK\r\n");
        while(1){} // get stuck here if the chip does not ACK back
    }
}

unsigned char i2c1_master_recv(void) { // receive a byte from the slave
    I2C1CONbits.RCEN = 1; // start receiving data
    while (!I2C1STATbits.RBF) {
        ;
    } // wait to receive the data
    return I2C1RCV; // read and return the data
}

void i2c1_master_ack(int val) { // sends ACK = 0 (slave should send another byte)
    // or NACK = 1 (no more bytes requested from slave)
    I2C1CONbits.ACKDT = val; // store ACK/NACK in ACKDT
    I2C1CONbits.ACKEN = 1; // send ACKDT
    while (I2C1CONbits.ACKEN) {
        ;
    } // wait for ACK/NACK to be sent
}

void i2c1_master_stop(void) { // send a STOP:
    I2C1CONbits.PEN = 1; // comm is complete and master relinquishes bus
    while (I2C1CONbits.PEN) {
        ;
    } // wait for STOP to complete
}


//==========================i2c2 funcs for IMU.==========================
void i2c2_master_setup(void) {
    // using a large BRG to see it on the nScope, make it smaller after verifying that code works
    // look up TPGD in the datasheet
        ANSELA = 0;     // Turn of analog pins
            ANSELB = 0;     // Turn of analog pins

        I2C2BRG = 1000; // I2CBRG = [1/(2*Fsck) - TPGD]*Pblck - 2 (TPGD is the Pulse Gobbler Delay) 100khz
    I2C2CONbits.ON = 1; // turn on the I2C2 module
}

void i2c2_master_start(void) {
    I2C2CONbits.SEN = 1; // send the start bit
    while (I2C2CONbits.SEN) {
        ;
                LATAINV = 0x0010;           // invert LATA4 value for a heart beat LED

    } // wait for the start bit to be sent
}

void i2c2_master_restart(void) {
    I2C2CONbits.RSEN = 1; // send a restart 
    while (I2C2CONbits.RSEN) {
        ;
    } // wait for the restart to clear
}

void i2c2_master_send(unsigned char byte) { // send a byte to slave
    I2C2TRN = byte; // if an address, bit 0 = 0 for write, 1 for read
    while (I2C2STATbits.TRSTAT) {
        ;
    } // wait for the transmission to finish
    if (I2C2STATbits.ACKSTAT) { // if this is high, slave has not acknowledged
        // ("I2C2 Master: failed to receive ACK\r\n");
        while(1){} // get stuck here if the chip does not ACK back
    }
}

unsigned char i2c2_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1; // start receiving data
    while (!I2C2STATbits.RBF) {
        ;
    } // wait to receive the data
    return I2C2RCV; // read and return the data
}

void i2c2_master_ack(int val) { // sends ACK = 0 (slave should send another byte)
    // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val; // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1; // send ACKDT
    while (I2C2CONbits.ACKEN) {
        ;
    } // wait for ACK/NACK to be sent
}

void i2c2_master_stop(void) { // send a STOP:
    I2C2CONbits.PEN = 1; // comm is complete and master relinquishes bus
    while (I2C2CONbits.PEN) {
        ;
    } // wait for STOP to complete
}


////---------------------------------------------------------Mid level funcs-------------------------
//void send_full_command(unsigned char address, unsigned char val){
//    const unsigned char write_address = 0b01000000; 
//    i2c1_master_start(); 
//    i2c1_master_send(write_address); 
//    i2c1_master_send(address);
//    i2c1_master_send(val); 
//    i2c1_master_stop(); 
//}
//
//unsigned char read_pin(unsigned char GPIO_address){
//    const unsigned char write_address = 0b01000000;
//    const unsigned char read_address = 0b01000001; 
//    i2c1_master_start();                     //send a start bit
//    i2c1_master_send(write_address);         //write to the write register in MCP that you want to read from the GPIO address
//    i2c1_master_send(GPIO_address);
//    i2c1_master_restart(); 
//    i2c1_master_send(read_address);          // start to read from the read register in MCP
//    unsigned char ret_val = i2c1_master_recv(); //waiting for the returned byte
//    i2c1_master_ack(1); 
//    i2c1_master_stop(); 
//    
//    return ret_val; 
//}
//
////---------------------------------------------------------Higher level funcs-------------------------
//void configure_MCP_IO(void){
//
//    const unsigned char IODIRA_ADDRESS = 0x00;
//    send_full_command(IODIRA_ADDRESS, 0x00);     // send 0x00 to IODIRA so all A pins are outputs
//    
//    const unsigned char IODIRB_ADDRESS = 0x01;  
//    send_full_command(IODIRB_ADDRESS, 0xFF);     // send 0xFF to IODIRB so all B pins are inputs
//}
//
//void invert_LED(void){
//    static char LED_VAL = 0; 
//    const unsigned char OLATA_ADDRESS = 0x14;
//    const unsigned char GPA7_VAL = LED_VAL<<7 | 0b0000000; // LED is GPA7
//    send_full_command(OLATA_ADDRESS, GPA7_VAL); 
//    LED_VAL = (LED_VAL == 0)?1:0; 
//}   
//
//unsigned char if_button_pressed (void){
//    const unsigned char GPB_ADDRESS = 0x13;        //by default, the MCP is in 8 bit mode and this is the GPB address. 
//    unsigned char GPB0_val = read_pin(GPB_ADDRESS) & 0b00000001; 
////    unsigned char GPB0_val = read_pin(GPB_ADDRESS); 
//
//    return (GPB0_val==0)?1:0; 
//}