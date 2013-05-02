#ifndef __SPI_DRIVER_H__
#define __SPI_DRIVER_H__

#include <stdio.h>
#include "paper.h"

extern void spi_setup(void);
extern void epClear(unsigned char color);
extern void EpStartSend(void);
extern void EpStopSend(void);
extern void epSendData(uartBuff *buff);

#endif