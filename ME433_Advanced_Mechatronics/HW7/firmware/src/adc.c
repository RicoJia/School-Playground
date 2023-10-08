#include "adc.h"

#define SAMPLE_TIME 10 // in core timer ticks, use a minimum of 250 ns

unsigned int adc_sample_convert(int pin) {
    unsigned int elapsed = 0, finish_time = 0;
    AD1CHSbits.CH0SA = pin; // connect chosen pin to MUXA for sampling
    AD1CON1bits.SAMP = 1; // start sampling
    elapsed = _CP0_GET_COUNT();
    finish_time = elapsed + SAMPLE_TIME;
    while (_CP0_GET_COUNT() < finish_time) {
        ; // sample for more than 250 ns
    }
    AD1CON1bits.SAMP = 0; // stop sampling and start converting
    while (!AD1CON1bits.DONE) {
        ; // wait for the conversion process to finish
    }
    return ADC1BUF0; // read the buffer with the result
}

void adc_setup() {
    ANSELBbits.ANSB3 = 1; // set analog pins with ANSEL      //TODO add another pin 

    AD1CON3bits.ADCS = 1; // ADC clock period is Tad = 2*(ADCS+1)*Tpb = 2*2*20.3ns = 83ns > 75ns
    IEC0bits.AD1IE = 0; // Disable ADC interrupts
    AD1CON1bits.ADON = 1; // turn on A/D converter
}

void ctmu_setup() {
    // base level current is about 0.55uA
    CTMUCONbits.IRNG = 0b11; // 100 times the base level current
    CTMUCONbits.ON = 1; // Turn on CTMU

    // 1ms delay to let it warm up
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < 48000000 / 2 / 1000) {
    }
}

int ctmu_read(int pin, int delay) {     //pass 5 into pin
    int start_time = 0;
    AD1CHSbits.CH0SA = pin;
    AD1CON1bits.SAMP = 1; // Manual sampling start
    CTMUCONbits.IDISSEN = 1; // Ground the pin
    // Wait 1 ms for grounding
    start_time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < start_time + 48000000 / 2 / 1000) {
    }
    CTMUCONbits.IDISSEN = 0; // End drain of circuit

    CTMUCONbits.EDG1STAT = 1; // Begin charging the circuit
    // wait delay core ticks
    start_time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < start_time + delay) {
    }
    AD1CON1bits.SAMP = 0; // Begin analog-to-digital conversion
    CTMUCONbits.EDG1STAT = 0; // Stop charging circuit
    while (!AD1CON1bits.DONE) // Wait for ADC conversion
    {}
    AD1CON1bits.DONE = 0; // ADC conversion done, clear flag
    return ADC1BUF0; // Get the value from the ADC
}



//========================================helper function =====================================================

/// \brief helper function to calculate average adc value, integer value [0, 1024]. 
/// \param adc_ret_buffer for storing buffer for adc. 
/// \param adc_buffer_size size of the buffer
/// \return integer averate of adc_ret buffer
int get_average_adc_val(int* adc_ret_buffer, unsigned short int adc_buffer_size){
    unsigned short int i = 0; 
    int sum = 0; 
    for(; i<adc_buffer_size; ++i){
        sum+= i[adc_ret_buffer];        //I'm just showing off, hehe
    }
    return sum/adc_buffer_size; 
}