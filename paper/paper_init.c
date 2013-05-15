#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "paper.h"
#include "paper_init.h"

void clock_setup(void)
{
	//rcc_clock_setup_in_hse_12mhz_out_72mhz();
	rcc_clock_setup_in_hse_8mhz_out_24mhz();
	/* Enable GPIOA, GPIOB, GPIOC clock. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
				    RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);
	//UARTs
#ifndef PRODUCTION
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
#endif
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
	//enable backup register
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
	/* Enable SPI2 Periph and gpio clocks */
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
				    RCC_APB1ENR_SPI2EN);
	//Timer2
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
}

void gpio_setup(void)
{
	/* Set GPIO1 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1 | /*GPIO2 |*/ GPIO4 | GPIO12);
#ifdef PRODUCTION
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO8 | GPIO9 | GPIO10);
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO9);
#endif
}
