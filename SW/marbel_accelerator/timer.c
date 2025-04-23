/*
 * timer.c
 *
 */

#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer.h"


typedef enum timermode_e{
    normalmode = 0,
    pwmmode = 1,
    ctcmode = 2,
    fastpwmmode = 3
    }timermode_e;

//typedef enum timermode_e timermode_e;

static inline void timer_8bit_set_prescaler(uint16_t prescaler)
{
    TCCR2B &= 0xF8;
    switch(prescaler)
    {
        case 0:
            break;
        case 1:
            TCCR2B |= (1<<CS20);
            break;
        case 8:
            TCCR2B |= (1<<CS21);
            break;
        case 32:
            TCCR2B |= ((1<<CS21) | (1<<CS20));
            break;
        case 64:
            TCCR2B |= (1<<CS22);
            break;
        case 128:
            TCCR2B |= ((1<<CS22) | (1<<CS20));
            break;
        case 256:
            TCCR2B |= ((1<<CS22) | (1<<CS21));
            break;
        case 1024:
            TCCR2B |= ((1<<CS22) | (1<<CS21) | (1<<CS20));
            break;
        default:
            break;
    }
}

static inline void timer_8bit_set_mode(timermode_e mode)
{
    switch(mode)
    {
        case normalmode:
            break;
        case pwmmode:
            TCCR2A |= (1<<WGM20);
            break;
        case ctcmode:
            TCCR2A |= (1<<WGM21);
            break;
        case fastpwmmode:
            TCCR2A |= ((1<<WGM20) | (1<<WGM21));
            break;
        default:
            break;
    }
}

static inline void timer_8bit_enable_ovint(void)
{
    TIMSK2 |= (1<<TOIE2);
}


static inline void timer_8bit_disable_ovint(void)
{
    TIMSK2 &= ~(1<<TOIE2);
}

static inline void timer_8bit_enable_compint(void)
{
    TIMSK2 |= (1<<OCIE2A);
}

static inline void timer_8bit_disable_compint(void)
{
    TIMSK2 &= ~(1<<OCIE2A);
}

static inline void timer_8bit_set_comp_reg(uint8_t compwert)
{
    OCR2A = compwert;
}

static inline uint8_t timer_8bit_get_comp_flag(void)
    {return (TIFR2 & (1<<OCF2A));}
static inline void timer_8bit_del_comp_flag(void)
    {(TIFR2 |= (1<<OCF2A));}


#define TIMER_8BIT_OV_ISR()        ISR(TIMER2_OVF_vect)
#define TIMER_8BIT_COMP_ISR()    ISR(TIMER2_COMPA_vect)

#define TIMER_PRESCALER    64
#define TIMER_COMP_WERT 249


static volatile uint16_t systick;

TIMER_8BIT_COMP_ISR()
{
    systick++;
}


void timer_init(void)
{
    systick = 0;
    timer_8bit_set_mode(ctcmode);
    timer_8bit_set_comp_reg(TIMER_COMP_WERT);
    timer_8bit_enable_compint();
    timer_8bit_set_prescaler(TIMER_PRESCALER);
}

uint16_t timer_get_systicks_ms(void)
{
    uint16_t tmp_systick;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        tmp_systick = systick;
    }
    return tmp_systick;
}
