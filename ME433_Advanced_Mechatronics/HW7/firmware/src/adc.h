#ifndef ADC_H__
#define ADC_H__

#include <xc.h>

void adc_setup();
unsigned int adc_sample_convert(int pin);

void ctmu_setup();
int ctmu_read(int pin, int delay);


//========================================helper function =====================================================

/// \brief helper function to calculate average adc value, integer value [0, 1024]. 
/// \param adc_ret_buffer for storing buffer for adc. 
/// \param adc_buffer_size size of the buffer
/// \return integer averate of adc_ret buffer


int get_average_adc_val(int* adc_ret_buffer, unsigned short int adc_buffer_size); 

#endif