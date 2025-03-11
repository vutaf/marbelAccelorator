/*
 * timer.h
 *
 */


#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

void timer_init(void);
uint16_t timer_get_systicks_ms(void);


#endif /* TIMER_H */