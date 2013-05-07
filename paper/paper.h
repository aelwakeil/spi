#ifndef __PAPER_H__
#define __PAPER_H__


void delay_ms(int d);
//#define	EP_BUFF_SIZE	1
#define	EP_BUFF_SIZE	80000
#define	EP_BYTES	163796//163840
//#define		EP_ROUND_BUF	40000

//#define		EP_ROUND_BUF	1

typedef enum {
  DATAMODE, CONFIGMODE, IDDLE
} uart_mode;

typedef struct uart_buff
{
  u8 buf[EP_BUFF_SIZE];
  int start;
  int pointer;
  int p;
  int rdy;
  int owcounter;
  int complete;
  uart_mode uartMode;
} uartBuff;


/*

static u8 bbuff[EP_ROUND_BUF];
static int bbufCounter;
static int bbufReaded;*/

#endif