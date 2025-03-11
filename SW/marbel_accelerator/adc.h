/*
 * adc.h
 */
#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void ADCInit(void);
uint16_t ADCRead(uint8_t kanal);
uint16_t ADCRead_raw(uint8_t kanal);

#endif /* ADC_H */