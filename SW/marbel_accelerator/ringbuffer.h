/*
 * ringpuffer.h
 *
 */

#ifndef RINGPUFFER_H
#define RINGPUFFER_H

#include <stdint.h>
#include <util/atomic.h>


#define BUFFER_SIZE    128
#define BUFFER_DATA_TYPE    uint8_t

typedef struct{
    uint8_t head;
    uint8_t tail;
    BUFFER_DATA_TYPE buffer[BUFFER_SIZE];
}s_buffer;


void buffer_init(s_buffer *buffer);
uint8_t buffer_store_data(BUFFER_DATA_TYPE data, volatile s_buffer *buffer);
uint8_t buffer_get_data(BUFFER_DATA_TYPE *data, volatile s_buffer *buffer);
uint8_t buffer_data_available(volatile s_buffer *buffer);

#endif /* RINGPUFFER_H */