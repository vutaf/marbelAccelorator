/*
 * main.c
 *
 */

#include <avr/interrupt.h>
#include "uart.h"
#include "accelerator.h"


#define BAUD_RATE 250000    //be aware, this is not a standard baudrate, as standard baurate will produce errors in timings with a 16Mhz Sysclock


int main(void)
{
    uart_init(BAUD_RATE);
    accelerator_init();

    sei();                  //global enable interrupts

    uart_send_P("Start accelerator\n");

    for(;;)
    {
        accelerator_task();
    }
}
