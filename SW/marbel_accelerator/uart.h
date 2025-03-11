/*
 * uart.h
 *
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "ringbuffer.h"

#include "uart_config.h"

#define uart_send_P(__s)    uart_send_progmem_string(PSTR(__s))

#define UART_RX_BUFFER_OV 1

extern void uart_init(uint32_t baudrate);

extern uint8_t uart_send_byte(uint8_t data);

extern uint8_t uart_get_byte(uint8_t *data);

uint8_t uart_send_string(uint8_t *data);

uint8_t uart_send_progmem_string(const char *progmem_data);

#endif /* UART_H */
