#ifndef __ADC_DRIVER_H__
#define __ADC_DRIVER_H__

#include "paper.h"
typedef enum {
  BATTOK, BATTLOW, BATTCRIT
} batt_state;

extern void adc_setup(void);
extern u16 adc_measure(void);
batt_state adc_testBatt(void);


#endif