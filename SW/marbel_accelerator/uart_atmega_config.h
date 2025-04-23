/*
 * uart_atmega_config.h
 *
 */


#ifndef UART_ATMEGA_CONFIG_H
#define UART_ATMEGA_CONFIG_H

#if defined(__AVR_ATmega165P__)

static inline void uart_config_set_8N1(void)        //8Bit, no Parity, 1Stopbit
    {UCSR0C |= ((1<<UCSZ01) | (1<<UCSZ00));}

static inline void uart_config_enable_tx(void)
    {UCSR0B |= (1<<TXEN0);}

static inline void uart_config_enable_rx(void)
    {UCSR0B |= (1<<RXEN0);}

static inline void uart_tx_enable_int(void)
    {UCSR0B |= (1<<UDRIE0);}

static inline void uart_tx_disable_int(void)
    {UCSR0B &= ~(1<<UDRIE0);}

static inline void uart_rx_enable_int(void)
    {UCSR0B |= (1<<RXCIE0);}

static inline void uart_transmit_byte(uint8_t data)
    {UDR0 = data;}

static inline uint8_t uart_receive_byte(void)
    {return UDR0;}

static inline void uart_set_baudrate_register(uint16_t regval)
{
    UBRR0H = (uint8_t)(regval>>8);
    UBRR0L = (uint8_t) regval;
}

#define UART_TX_ISR()    ISR(USART0_UDRE_vect)
#define UART_RX_ISR()    ISR(USART0_RX_vect)

#elif defined(__AVR_ATmega32U4__)

static inline void uart_config_set_8N1(void)        //8Bit, no Parity, 1Stopbit
    {UCSR1C |= ((1<<UCSZ11) | (1<<UCSZ10));}

static inline void uart_config_enable_tx(void)
    {UCSR1B |= (1<<TXEN1);}

static inline void uart_config_enable_rx(void)
    {UCSR1B |= (1<<RXEN1);}

static inline void uart_tx_enable_int(void)
    {UCSR1B |= (1<<UDRIE1);}

static inline void uart_tx_disable_int(void)
    {UCSR1B &= ~(1<<UDRIE1);}

static inline void uart_rx_enable_int(void)
    {UCSR1B |= (1<<RXCIE1);}

static inline void uart_transmit_byte(uint8_t data)
    {UDR1 = data;}

static inline uint8_t uart_receive_byte(void)
    {return UDR1;}

static inline void uart_set_baudrate_register(uint16_t regval)
{
    UBRR1H = (uint8_t)(regval>>8);
    UBRR1L = (uint8_t) regval;
}

#define UART_TX_ISR()    ISR(USART1_UDRE_vect)
#define UART_RX_ISR()    ISR(USART1_RX_vect)

#elif defined(__AVR_ATtiny1634__)

static inline void uart_config_set_8N1(void)        //8Bit, no Parity, 1Stopbit
    {UCSR0C |= ((1<<UCSZ01) | (1<<UCSZ00));}

static inline void uart_config_enable_tx(void)
    {UCSR0B |= (1<<TXEN0);}

static inline void uart_config_enable_rx(void)
    {UCSR0B |= (1<<RXEN0);}

static inline void uart_tx_enable_int(void)
    {UCSR0B |= (1<<UDRIE0);}

static inline void uart_tx_disable_int(void)
    {UCSR0B &= ~(1<<UDRIE0);}

static inline void uart_rx_enable_int(void)
    {UCSR0B |= (1<<RXCIE0);}

static inline void uart_transmit_byte(uint8_t data)
    {UDR0 = data;}

static inline uint8_t uart_receive_byte(void)
    {return UDR0;}

static inline void uart_set_baudrate_register(uint16_t regval)
{
    UBRR0H = (uint8_t)(regval>>8);
    UBRR0L = (uint8_t) regval;
}

#define UART_TX_ISR()    ISR(USART0_UDRE_vect)
#define UART_RX_ISR()    ISR(USART0_RX_vect)

#elif defined(__AVR_ATmega168PA__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88A__)

static inline void uart_config_set_8N1(void)        //8Bit, no Parity, 1Stopbit
    {UCSR0C |= ((1<<UCSZ01) | (1<<UCSZ00));}

static inline void uart_config_enable_tx(void)
    {UCSR0B |= (1<<TXEN0);}

static inline void uart_config_enable_rx(void)
    {UCSR0B |= (1<<RXEN0);}

static inline void uart_tx_enable_int(void)
    {UCSR0B |= (1<<UDRIE0);}

static inline void uart_tx_disable_int(void)
    {UCSR0B &= ~(1<<UDRIE0);}

static inline void uart_txc_enable_int(void)
    {UCSR0B |= (1<<TXCIE0);}

static inline void uart_txc_disable_int(void)
    {UCSR0B &= ~(1<<TXCIE0);}

static inline void uart_rx_enable_int(void)
    {UCSR0B |= (1<<RXCIE0);}

static inline void uart_transmit_byte(uint8_t data)
    {UDR0 = data;}

static inline uint8_t uart_receive_byte(void)
    {return UDR0;}

static inline void uart_set_baudrate_register(uint16_t regval)
{
    UBRR0H = (uint8_t)(regval>>8);
    UBRR0L = (uint8_t) regval;
}

#define UART_TX_ISR()    ISR(USART_UDRE_vect)
#define UART_RX_ISR()    ISR(USART_RX_vect)
#define UART_TX_COMPLETE_ISR()    ISR(USART_TX_vect)


#elif defined(__AVR_ATmega640__)

static inline void uart_config_set_8N1(void)        //8Bit, no Parity, 1Stopbit
    {UCSR0C |= ((1<<UCSZ01) | (1<<UCSZ00));}        //8Bit

static inline void uart_config_enable_tx(void)
    {UCSR0B |= (1<<TXEN0);}                            //transmitter enable

static inline void uart_config_enable_rx(void)
    {UCSR0B |= (1<<RXEN0);}                            //receiver enable

static inline void uart_tx_enable_int(void)
    {UCSR0B |= (1<<UDRIE0);}                        //enables interrupt on the UDRE0 Flag

static inline void uart_tx_disable_int(void)
    {UCSR0B &= ~(1<<UDRIE0);}

static inline void uart_rx_enable_int(void)
    {UCSR0B |= (1<<RXCIE0);}

static inline void uart_transmit_byte(uint8_t data)
    {UDR0 = data;}                                    //transmit byte

static inline uint8_t uart_receive_byte(void)
    {return UDR0;}                                    //receive byte

static inline void uart_set_baudrate_register(uint16_t regval)
{
    UBRR0H = (uint8_t)(regval>>8);
    UBRR0L = (uint8_t) regval;
}

#define UART_TX_ISR()    ISR(USART0_UDRE_vect)
#define UART_RX_ISR()    ISR(USART0_RX_vect)

#elif defined(__AVR_ATmega8515__)

static inline void uart_config_set_8N1(void)        //8Bit, no Parity, 1Stopbit
{
    UCSRC = ((1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0));
}

static inline void uart_config_enable_tx(void)
    {UCSRB |= (1<<TXEN);}                            //transmitter enable

static inline void uart_config_enable_rx(void)
    {UCSRB |= (1<<RXEN);}                            //receiver enable

static inline void uart_tx_enable_int(void)
    {UCSRB |= (1<<UDRIE);}                        //enables interrupt on the UDRE0 Flag

static inline void uart_tx_disable_int(void)
    {UCSRB &= ~(1<<UDRIE);}

static inline void uart_rx_enable_int(void)
    {UCSRB |= (1<<RXCIE);}

static inline void uart_transmit_byte(uint8_t data)
    {UDR = data;}                                    //transmit byte

static inline uint8_t uart_receive_byte(void)
    {return UDR;}                                    //receive byte

static inline void uart_set_baudrate_register(uint16_t regval)
{
    UBRRH = (uint8_t)(regval>>8);
    UBRRL = (uint8_t) regval;
}

#define UART_TX_ISR()    ISR(USART_UDRE_vect)
#define UART_RX_ISR()    ISR(USART_RX_vect)

#elif defined(__AVR_ATtiny441__) || defined(__AVR_ATtiny841__)

static inline void uart_config_set_8N1(void)        //8Bit, no Parity, 1Stopbit
{UCSR0C |= ((1<<UCSZ01) | (1<<UCSZ00));}

static inline void uart_config_enable_tx(void)
{UCSR0B |= (1<<TXEN0);}

static inline void uart_config_enable_rx(void)
{UCSR0B |= (1<<RXEN0);}

static inline void uart_tx_enable_int(void)
{UCSR0B |= (1<<UDRIE0);}

static inline void uart_tx_disable_int(void)
{UCSR0B &= ~(1<<UDRIE0);}

static inline void uart_txc_enable_int(void)
{UCSR0B |= (1<<TXCIE0);}

static inline void uart_txc_disable_int(void)
{UCSR0B &= ~(1<<TXCIE0);}

static inline void uart_rx_enable_int(void)
{UCSR0B |= (1<<RXCIE0);}

static inline void uart_transmit_byte(uint8_t data)
{UDR0 = data;}

static inline uint8_t uart_receive_byte(void)
{return UDR0;}

static inline void uart_set_baudrate_register(uint16_t regval)
{
    UBRR0H = (uint8_t)(regval>>8);
    UBRR0L = (uint8_t) regval;
}

#define UART_TX_ISR()    ISR(USART0_UDRE_vect)
#define UART_RX_ISR()    ISR(USART0_RX_vect)
#define UART_TX_COMPLETE_ISR()    ISR(USART0_TX_vect)


#elif defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__)

static inline void uart_config_set_8N1(void)        //8Bit, no Parity, 1Stopbit
{UCSR0C |= ((1<<UCSZ01) | (1<<UCSZ00));}

static inline void uart_config_enable_tx(void)
{UCSR0B |= (1<<TXEN0);}

static inline void uart_config_enable_rx(void)
{UCSR0B |= (1<<RXEN0);}

static inline void uart_tx_enable_int(void)
{UCSR0B |= (1<<UDRIE0);}

static inline void uart_tx_disable_int(void)
{UCSR0B &= ~(1<<UDRIE0);}

static inline void uart_txc_enable_int(void)
{UCSR0B |= (1<<TXCIE0);}

static inline void uart_txc_disable_int(void)
{UCSR0B &= ~(1<<TXCIE0);}

static inline void uart_rx_enable_int(void)
{UCSR0B |= (1<<RXCIE0);}

static inline void uart_transmit_byte(uint8_t data)
{UDR0 = data;}

static inline uint8_t uart_receive_byte(void)
{return UDR0;}

static inline void uart_set_baudrate_register(uint16_t regval)
{
    UBRR0H = (uint8_t)(regval>>8);
    UBRR0L = (uint8_t) regval;
}

#define UART_TX_ISR()    ISR(USART0_UDRE_vect)
#define UART_RX_ISR()    ISR(USART0_RX_vect)
#define UART_TX_COMPLETE_ISR()    ISR(USART0_TX_vect)

#else
    #error used MCU is not supported
#endif


#endif /* UART_ATMEGA_CONFIG_H */
