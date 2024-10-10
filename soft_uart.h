#ifndef __SOFT_UART_H__
#define __SOFT_UART_H__

#include <avr/io.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <avr/pgmspace.h>

/**
 * Simple software UART implementation
 *
 **/

#define sUartTXPort PORTA
#define sUartTXDDRPort DDRA
#define sUartTXPin PA4

//extern uint16_t g_sUartDelayValue;

       
void sUartWriteBit( uint8_t a_ui8Bit);
void sUartDelay( uint16_t a_ui16Delay);
void sUartPrintChar( uint8_t a_ui8Byte);
void sUartPrintString( char *a_pString);
void sUartInit( unsigned long a_ul32BaudRate);
void sUartPrintf (const char *a_pBuffer,...);

#endif //__SOFT_UART_H__
