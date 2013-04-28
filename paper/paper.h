#ifndef __PAPER_H__
#define __PAPER_H__

void delay_ms(int d);
#define	EP_BUFF_SIZE	40960
//#define	EP_BUFF_SIZE	163840

typedef struct uart_buff
{
  u8 buf[EP_BUFF_SIZE];
  int pointer;
  int rdy;
  int owcounter;
} uartBuff;


#endif