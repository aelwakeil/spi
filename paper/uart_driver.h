#ifndef __UART_DRIVER_H__
#define __UART_DRIVER_H__

#include "paper.h"

extern void usart_setup(uartBuff *ubuff);

extern int SendChar (int ch);
extern void uart_printf (char *ptr);

#endif