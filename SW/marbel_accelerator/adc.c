/*
 * adc.c
 *
 */

#include <avr/io.h>
#include "adc.h"

typedef enum adc_ref_e{
    ref_extern = 0,
    ref_avcc = 1,
    ref_intern = 3
}adc_ref_e;



static inline void adc_set_reference(adc_ref_e rev)
{
    ADMUX &= 0x3F;        //clear old rev
    ADMUX |= (rev << 6);
}


static inline void adc_set_prescaler(uint8_t prescaler)
{
    if (prescaler < 9)
    {
        ADCSRA &= 0xF8;        //clear old prescaler
        ADCSRA |= prescaler;
    }
}

static inline void adc_set_channel(uint8_t channel)
{
    ADMUX &= 0xE0;                //clear old channel
    ADMUX |= (channel & 0x1F);
}

static inline void adc_enable(void)
    {ADCSRA |= (1<<ADEN);}

static inline void adc_disable(void)
    {ADCSRA &= ~(1<<ADEN);}

static inline void adc_start_conversion(void)
    {ADCSRA |= (1<<ADSC);}

static inline uint8_t adc_get_conversion_flag(void)
{
    return (ADCSRA & (1<<ADSC));
}

static inline uint16_t adc_get_conversion_result_10bit(void)
{
    uint16_t tmp;
    tmp = (uint16_t) ADCL;
    tmp |= (((uint16_t) ADCH)<<8);
    return tmp;
}

static inline uint8_t adc_get_conversion_result_8bit(void)    //achtung, setzt voraus, das das ergebniss linksbuendig ist
{
    return ADCH;
}


#define NUMBER_OF_CONVERSIONS   8

void ADCInit(void)
{
    adc_set_reference(ref_avcc);
    adc_set_prescaler(6);
    adc_enable();
    adc_set_channel(0x1E);
    adc_start_conversion();
    while(adc_get_conversion_flag())
        {}
    adc_get_conversion_result_10bit();
}

uint16_t ADCRead_raw(uint8_t kanal)
{
    uint16_t result = 0;
    adc_set_channel(kanal);
    adc_start_conversion();
    while(adc_get_conversion_flag())
        {}
    result = adc_get_conversion_result_10bit();
    return result;
}

uint16_t ADCRead(uint8_t kanal)
{

    uint8_t i;
    uint16_t result = 0;
    adc_set_channel(kanal);

    //Dummy-Readout
    adc_start_conversion();
    while(adc_get_conversion_flag())
    {}

    for(i=0;i<NUMBER_OF_CONVERSIONS;i++)
    {
        adc_start_conversion();
        while(adc_get_conversion_flag())
            {}
        result += adc_get_conversion_result_10bit();
    }
    result /= NUMBER_OF_CONVERSIONS;
    return result;
}
