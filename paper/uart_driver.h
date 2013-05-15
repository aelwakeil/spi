#ifndef __UART_DRIVER_H__
#define __UART_DRIVER_H__

#include "paper.h"

#define EXIT_CHAR 'E'

extern void usart_setup(uartBuff *ubuff);

extern int SendChar (u32 usart, int ch);
extern void uart_printf (u32 usart, char *ptr);
extern void uart_print_int(u32 usart, int value);


#endif