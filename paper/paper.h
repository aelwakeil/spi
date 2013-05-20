#ifndef __PAPER_H__
#define __PAPER_H__

#include <stdio.h>
#include <errno.h>

void delay_ms(int d);
//#define	EP_BUFF_SIZE	1
#define	EP_BUFF_SIZE	80000
#define	EP_BYTES	163796//163840
//#define		EP_ROUND_BUF	40000
#define BT_NAME_LENGTH	10


//#define VERBOSE
#define PRODUCTION


//#define		EP_ROUND_BUF	1

typedef enum {
  IDDLE = 1, DATAMODE, CONFIGMODE, CONFIGBTMODE
} uart_mode;

typedef struct uart_buff
{
  uart_mode uartMode;
  u8 buf[EP_BUFF_SIZE];
  int start;
  int pointer;
  int p;
  int rdy;
  int owcounter;
  int complete;
} uartBuff;

typedef struct settings
{
  u8   deviceId;
  char btName[BT_NAME_LENGTH];
  char btPin[8];
  u8   btPinLen;
  u8  iddleTime;
} paperSettings;
/*

static u8 bbuff[EP_ROUND_BUF];
static int bbufCounter;
static int bbufReaded;*/

#endif