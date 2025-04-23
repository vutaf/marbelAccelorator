/*
 * accelerator.c
 *
 */

/*
 * Config: Hallsensor 0 on ADC7 (no GPIO)
 *         Hallsensor 1 on PC0
 *         CNY70 on PC1
 *         CNY70 enable on PD5 (COM0B)
 *         switch on PC0    -> removed
 *         led1 on PD6
 *         led2 on PD7
 *         coil 0 on PC3
 *         coil 1 on PC2
 *         debug pin on PD3
 *         debug pin2 on PD4
*/
#include <avr/interrupt.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>        //for utoa
#include <stdbool.h>
#include <util/delay.h>
#include "accelerator.h"
#include "adc.h"
#include "uart.h"
#include "timer.h"

#define NUMBER_OFF_COILS    2
#define MAX_ACTIVE_TIME_COIL0_MS  800
#define MAX_ACTIVE_TIME_COIL1_MS  200
#define DELAY_BETWEEN_TWO_COILS_MS 2    //delay in ms between deactivation of coil0 and activation of coil1,
                                        //to ensure undisturbed measurement of no field voltage on coil1
#define DEFAULT_NO_FIELD_DIGITS 511
#define ACTIVATION_THRESHOLD    512     //Threshold on trigger input -> ~2,5V
#define ADC_BUFFER_SIZE 4   //max 64, otherwise there will occur overflow

#define ACCELERATOR_TIMER_COMPARE_VALUE 200 //in combination with 2Mhz Timer ->10kHz samplefrequenz

#define MINABRRUCH 180
#define THRESHOLD 2



typedef enum {
    ACCELERATOR_WAIT_FOR_MARBEL,
    ACCELERATOR_FIRST_STAGE_ACTIVE,
    ACCELERATOR_SECOND_STAGE_ACTIVE,
    ACCELERATOR_BLIND_TIME
} acceleratorState_t;

typedef struct {
    uint16_t noField;
    uint16_t coilActiv;
    uint16_t maxAccField;
} hallVoltages_t;

typedef struct {
    uint16_t data[ADC_BUFFER_SIZE];
    uint8_t nextSlot;
} adcRingBuffer_t;

typedef struct {
    acceleratorState_t currentState;
    hallVoltages_t coilVoltages[NUMBER_OFF_COILS];
    uint8_t adcChannelHall[NUMBER_OFF_COILS];
    uint8_t adcChannelTrigger;
    uint16_t activationStartTimestamp;
    uint8_t activeCoil;
    volatile bool accCycleFinished;
} acceleratorHandle_t;

/* privat function declaration ********************************************/

static inline void debugPin_set(void);
static inline void debugPin_clr(void);
static inline void debugPin_tog(void);

static inline void debugPin2_set(void);
static inline void debugPin2_clr(void);
static inline void debugPin2_tog(void);

static inline void activateAutotrigger(void);
static inline void deactivateAutotrigger(void);

static inline void activateAdcInt(void);
static inline void deactivateAdcInt(void);

static inline void activateCoil0(void);
static inline void deactivateCoil0(void);

static inline void activateCoil1(void);
static inline void deactivateCoil1(void);

static inline void deactivateCurrentCoil(void);

static inline void initAcceleratorTimer(void);

static void resetStateMachine(void);

static void startDefaultstate(void);

static void initFieldMeasurementAndTimeout(void);

bool accelerator_getSwitchState(void);

void accelerator_setLed1(void);
void accelerator_clrLed1(void);

void accelerator_setLed2(void);
void accelerator_clrLed2(void);

//attention: designed for handling of 10Bit adc values
void ringbuffer_setValue(uint16_t data);
uint16_t ringbuffer_getSumme(void);

/* modul variables declaration ********************************************/

static acceleratorHandle_t handle;

static adcRingBuffer_t adcBuffer;
static uint16_t maxField;

static uint16_t ticks;

/* ISR functions **********************************************************/

ISR(ADC_vect)
{
    // debugPin_set();
    debugPin2_set();

    uint16_t adcValue;
    adcValue = (uint16_t) ADCL;
    adcValue |= (((uint16_t) ADCH)<<8);
    if (handle.currentState == ACCELERATOR_WAIT_FOR_MARBEL)
    {
        if (adcValue > ACTIVATION_THRESHOLD)
        {
            deactivateAutotrigger();
            handle.accCycleFinished = true;
        }
    }
    else
    {
        uint16_t adcValueAbsolute;
        if (adcValue > handle.coilVoltages[handle.activeCoil].noField)
        {
            adcValueAbsolute = (adcValue - handle.coilVoltages[handle.activeCoil].noField);
        }
        else
        {
            adcValueAbsolute = (handle.coilVoltages[handle.activeCoil].noField - adcValue);
        }

        if (adcValueAbsolute > maxField)
        {
            maxField = adcValueAbsolute;
        }

        if (adcValueAbsolute > MINABRRUCH)
        {
            if ((adcValueAbsolute + THRESHOLD) < maxField)
            {
                deactivateCurrentCoil();
                deactivateAutotrigger();
                deactivateAdcInt();
                handle.accCycleFinished = true;
            }
        }
        // uart_send_byte((uint8_t)(adcValue >> 2));
    }
    debugPin2_clr();
}

EMPTY_INTERRUPT(TIMER0_COMPA_vect)   //nothing to do here, we need the ISR to clear the int flag

/* public functions implementation ***************************************/

void accelerator_init(void)
{
    timer_init();

    handle.currentState = ACCELERATOR_WAIT_FOR_MARBEL;
    handle.coilVoltages[0].noField = DEFAULT_NO_FIELD_DIGITS;
    handle.coilVoltages[0].maxAccField = 230;   //measured max field: 240
    handle.coilVoltages[1].noField = DEFAULT_NO_FIELD_DIGITS;
    handle.coilVoltages[1].maxAccField = 190;   //measured max field: 200
    handle.activeCoil = 0;
    handle.accCycleFinished = false;
    handle.adcChannelHall[0] = 7;
    handle.adcChannelHall[1] = 0;
    handle.adcChannelTrigger = 1;
    uart_send_P("Acc init\n");


    DDRD |= (1<<PIND6);        //set led1 pin as output
    DDRD |= (1<<PIND7);        //set led2 pin as output

    DDRC |= (1<<PINC3);        //set coil0 pin as output
    DDRC |= (1<<PINC2);        //set coil1 pin as output
    DDRD |= (1<<PIND3);        //set debug pin as output
    debugPin_clr();
    DDRD |= (1<<PIND4);        //set debug pin2 as output
    debugPin2_clr();

    //set hall sensor 0 pin is only analog, so nothing to do

    DDRC &= ~(1<<PINC0);    //set hall sensor 1 pin to input (should be default)
    DIDR0 |= (1<<ADC0D);    //deactivate digital buffer on PC0

    DDRC &= ~(1<<PINC1);    //set CNY70 pin to input (should be default)
    DIDR0 |= (1<<ADC1D);    //deactivate digital buffer on PC1

    initAcceleratorTimer();

    ADCSRB |= ((1<<ADTS1) | (1<<ADTS0));    //set Timer0 comp match as adc trigger source

    ADCInit();
    startDefaultstate();
}


bool accelerator_getSwitchState(void)
{
    if (PINC & (1<<PINC0))
    {
        return false;
    }
    else
    {
        return true;
    }
}

void accelerator_setLed1(void)
{
    PORTD |= (1<<PIND6);
}

void accelerator_clrLed1(void)
{
    PORTD &= ~(1<<PIND6);
}

void accelerator_setLed2(void)
{
    PORTD |= (1<<PIND7);
}

void accelerator_clrLed2(void)
{
    PORTD &= ~(1<<PIND7);
}

char charBuffer[7];

void accelerator_task(void)
{
    switch(handle.currentState)
    {
    case ACCELERATOR_WAIT_FOR_MARBEL:
        if ((true == handle.accCycleFinished) || (true == accelerator_getSwitchState()))
        {
            debugPin_set();
            accelerator_setLed1();
            handle.activeCoil = 0;
            //measure current no field voltage for first coil
            initFieldMeasurementAndTimeout();
            //activate coil
            activateCoil0();
            //start field measurement
            activateAutotrigger();
            activateAdcInt();
            //switch to next state
            handle.currentState = ACCELERATOR_FIRST_STAGE_ACTIVE;
            uart_send_P("Activate first Coil\n");
        }
        break;
    case ACCELERATOR_FIRST_STAGE_ACTIVE:
        if ((timer_get_systicks_ms() - handle.activationStartTimestamp) > MAX_ACTIVE_TIME_COIL0_MS)
        {
            //ABORT
            resetStateMachine();
        }
        //regular end of state due ADC ISR for minimal timing, start next stage
        if (handle.accCycleFinished == true)
        {
            debugPin_clr();
            uart_send_P("End first Coil, start second\n");
            uart_send_P("MaxF0: ");
            utoa(maxField, charBuffer, 10);
            uart_send_string((uint8_t *)charBuffer);
            uart_send_P("\n");
            handle.activeCoil = 1;

            _delay_ms(DELAY_BETWEEN_TWO_COILS_MS);   //wait till field in coil0 is down, otherwise our "nofield"-value is bad...
            ticks = 0;
            initFieldMeasurementAndTimeout();

            handle.currentState = ACCELERATOR_SECOND_STAGE_ACTIVE;
            //activate coil
            ADCSRA |= (1<<ADIF);    //clear adc int flag
            debugPin_set();
            activateCoil1();
            activateAutotrigger();
            activateAdcInt();
        }

        break;
    case ACCELERATOR_SECOND_STAGE_ACTIVE:
        if ((timer_get_systicks_ms() - handle.activationStartTimestamp) > MAX_ACTIVE_TIME_COIL1_MS)
        {
            resetStateMachine();
            //ABORT
        }
        //regular end of state due ADC ISR for minimal timing
        if (handle.accCycleFinished == true)
        {
            handle.accCycleFinished = false;
            handle.activeCoil = 0;
            handle.currentState = ACCELERATOR_WAIT_FOR_MARBEL;
            debugPin_clr();
            accelerator_clrLed1();
            uart_send_P("End second Coil\n");
            uart_send_P("MaxF1: ");
            utoa(maxField, charBuffer, 10);
            uart_send_string((uint8_t *)charBuffer);
            uart_send_P("\n");
            //got to start
            startDefaultstate();

        }
        break;
    case ACCELERATOR_BLIND_TIME:
        //wait for coil cooldown
        break;
    default:
        break;
    }


}

/* privat functions implementation ***************************************/

static inline void activateCoil0(void)
{
    PORTC |= (1<<PINC3);
}

static inline void deactivateCoil0(void)
{
    PORTC &= ~(1<<PINC3);
}

static inline void activateCoil1(void)
{
    PORTC |= (1<<PINC2);
}

static inline void deactivateCoil1(void)
{
    PORTC &= ~(1<<PINC2);
}

static inline void deactivateCurrentCoil(void)
{
    switch (handle.activeCoil)
    {
        case 0:
            deactivateCoil0();
            break;
        case 1:
            deactivateCoil1();
            break;
        default:
            deactivateCoil0();
            deactivateCoil1();
            break;
    }
}

static inline void activateAutotrigger(void)
{
    ADCSRA |= (1<<ADATE);    //enable autotrigger
}

static inline void deactivateAutotrigger(void)
{
    ADCSRA &= ~(1<<ADATE);    //disable autotrigger
}

static inline void activateAdcInt(void)
{
    ADCSRA |= (1<<ADIE);    //enable adc int
}

static inline void deactivateAdcInt(void)
{
    ADCSRA &= ~(1<<ADIE);    //disable adc int
}

static inline void initAcceleratorTimer(void)
{
    TCCR0A |= (1<<WGM01);                //set timer0 to ctc mode
    // TCCR0B |= ((1<<CS01) | (1<<CS00));  //set prescaler to 32 -> timer run with 500kHz
    TCCR0B |= (1<<CS01);  //set prescaler to 8 -> timer run with 2MHz
    OCR0A = (ACCELERATOR_TIMER_COMPARE_VALUE - 1);  //set max value for timer0
    TIMSK0 |= (1<<OCIE0A);
}

static void startDefaultstate(void)
{

    uart_send_P("Start trigger");
    handle.currentState = ACCELERATOR_WAIT_FOR_MARBEL;
    handle.activeCoil = 0;
    handle.accCycleFinished = false;

    //set adc-channel to triggerchannel
    ADCRead(handle.adcChannelTrigger);
    ADCSRA |= (1<<ADIF);    //clear adc int flag
    //start trigger measurement
    activateAutotrigger();
    activateAdcInt();

}

static void resetStateMachine(void)
{
    deactivateCurrentCoil();
    deactivateAutotrigger();
    deactivateAdcInt();

    debugPin_clr();
    accelerator_clrLed1();

    uart_send_P("Timeout Coil: ");
    uart_send_byte('0'+ handle.activeCoil);
    uart_send_P("\n");

    startDefaultstate();
}

static void initFieldMeasurementAndTimeout(void)
{
    handle.coilVoltages[handle.activeCoil].noField = ADCRead(handle.adcChannelHall[handle.activeCoil]);
    maxField = 0;
    handle.accCycleFinished = false;
    handle.activationStartTimestamp = timer_get_systicks_ms();
    uart_send_P("NoF");
    uart_send_byte('0'+handle.activeCoil);
    uart_send_P(": ");
    utoa(handle.coilVoltages[handle.activeCoil].noField, charBuffer, 10);
    uart_send_string((uint8_t *)charBuffer);
    uart_send_P("\n");
}

inline static void debugPin_set(void)
{
    PORTD |= (1<<PIND3);
}

inline static void debugPin_clr(void)
{
    PORTD &= ~(1<<PIND3);
}

inline static void debugPin_tog(void)
{
    PORTD ^= (1<<PIND3);
}

inline static void debugPin2_set(void)
{
    PORTD |= (1<<PIND4);
}

inline static void debugPin2_clr(void)
{
    PORTD &= ~(1<<PIND4);
}

inline static void debugPin2_tog(void)
{
    PORTD ^= (1<<PIND4);
}

void ringbuffer_setValue(uint16_t data)
{
    adcBuffer.data[adcBuffer.nextSlot] = data;
    adcBuffer.nextSlot++;
    if (adcBuffer.nextSlot >= ADC_BUFFER_SIZE)
    {
        adcBuffer.nextSlot = 0;
    }
}

uint16_t ringbuffer_getSumme(void)
{
    uint16_t summe = 0;
    for (uint8_t i = 0; i < ADC_BUFFER_SIZE; i++)
    {
        summe += adcBuffer.data[i];
    }
    return summe;
}