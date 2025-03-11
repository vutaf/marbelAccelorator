/*
 * uart.c
 *
 */

#include "uart.h"


static volatile s_buffer tx_buffer;
static volatile s_buffer rx_buffer;
static volatile uint8_t currentError;

#ifdef WITH_UART_INT
    UART_TX_ISR()
    {
        if (buffer_data_available(&tx_buffer))
        {
            uint8_t tmp_data;
            buffer_get_data(&tmp_data, &tx_buffer);
            uart_transmit_byte(tmp_data);
        }
        else
        {
            uart_tx_disable_int();
        }
    }


    UART_RX_ISR()
    {
        uint8_t tmp_data;
        tmp_data = uart_receive_byte();

        if (!buffer_store_data(tmp_data, &rx_buffer))
            {}
        else
            {currentError |= UART_RX_BUFFER_OV;}
    }

#endif // WITH_UART_INT


void uart_init(uint32_t baudrate)
{
    baudrate = ((F_CPU)/((baudrate)*16l)-1);
    uart_set_baudrate_register((uint16_t)baudrate);
    uart_config_set_8N1();
    uart_config_enable_rx();
    uart_config_enable_tx();

    uart_rx_enable_int();

    buffer_init((s_buffer *) &tx_buffer);
    buffer_init((s_buffer *) &rx_buffer);
}

uint8_t uart_send_byte(uint8_t data)
{
#ifdef WITH_UART_INT
    if (!buffer_store_data(data, &tx_buffer))
    {
        uart_tx_enable_int();
        return 0;
    }
    else
        {return 1;}
#else
    while(!(UCSR0A & (1<<UDRE0)))
    {}
    uart_transmit_byte(data);
    return 0;
#endif
}

uint8_t uart_get_byte(uint8_t *data)
{
#ifdef WITH_UART_INT
    if (!buffer_get_data(data, &rx_buffer))
        {return 0;}
    else
        {return 1;}
#else
    if (UCSR0A & (1<<RXC0))
    {
        *data = UDR0;
        return 0;
    }
    else
    {
        return 1;
    }
#endif
}

uint8_t uart_send_string(uint8_t *data)
{
    while(*data)
    {
        if (uart_send_byte(*data))
            {return 1;}
        data++;
    }
    return 0;
}

uint8_t uart_send_progmem_string(const char *progmem_data)
{
    uint8_t tmp_byte;
    tmp_byte = pgm_read_byte(progmem_data);
    while(tmp_byte)
    {
        if (uart_send_byte(tmp_byte))
        {return 1;}
        progmem_data++;
        tmp_byte = pgm_read_byte(progmem_data);
    }
    return 0;

}
