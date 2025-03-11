/*
 * ringbuffer.c
 *
 */


#include "ringbuffer.h"

#define BUFFER_MASK    (BUFFER_SIZE - 1)


void buffer_init(s_buffer *buffer)
{
    buffer->head = 0;
    buffer->tail = 0;
}


uint8_t buffer_data_available(volatile s_buffer *buffer)
{
    if (buffer->head != buffer->tail)    //data in buffer
        {return 1;}
    else
        {return 0;}
}

uint8_t buffer_get_data(BUFFER_DATA_TYPE *data, volatile s_buffer *buffer)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        uint8_t tmp_tail;
        tmp_tail = buffer->tail;
        if (buffer->head != tmp_tail)    //data in buffer
        {
            *data = buffer->buffer[tmp_tail];
            tmp_tail = (tmp_tail + 1) & BUFFER_MASK;
            buffer->tail = tmp_tail;
            return 0;
        }
        else
            {return 1;}
    }
    return 1;   //if there is no return, the compiler will complain...
}

uint8_t buffer_store_data(BUFFER_DATA_TYPE data, volatile s_buffer *buffer)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        uint8_t tmp_head;
        tmp_head = buffer->head;
        if ((tmp_head + 1) != buffer->tail)
        {
            buffer->buffer[tmp_head] = data;
            tmp_head = (tmp_head + 1) & BUFFER_MASK;
            buffer->head = tmp_head;
            return 0;
        }
        else
            {return 1;}
    }
return 1;       //if there is no return, the compiler will complain...
}