/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2013 Stephen Dwyer <scdwyer@ualberta.ca>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include <errno.h>


void delay_ms(int d);
int SendChar (int ch);
void uart_printf (char *ptr);

static u8 data[50];
static int data_size = 0;
static int data_pointer = 0;

static u8 btBuff[1024];
static int btBuff_size = 0;
static int btBuff_pointer = 0;

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_24mhz();
	/* Enable GPIOA, GPIOB, GPIOC clock. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
				    RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);

	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	/*rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN |
				    RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN | RCC_APB1ENR_USART3EN);
*/	
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN);
	/* Enable SPI2 Periph and gpio clocks */
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
				    RCC_APB1ENR_SPI2EN);
}

static void gpio_setup(void)
{
	/* Set GPIO1 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1 | GPIO2 | GPIO4);
}

static void usart_setup(void)
{
  	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);
	
	/* Setup GPIO pin GPIO_USART1_TX and GPIO_USART1_RX. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	
	/* Setup UART parameters. */
	//usart_set_baudrate(USART1, 38400);
	USART_BRR(USART1) = (u16)((24000000 << 4) / (115200 * 16));
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	
	usart_disable_error_interrupt(USART1);
	usart_enable_rx_interrupt(USART1);
	usart_enable_tx_interrupt(USART1);	
	/* Finally enable the USART. */
	usart_enable(USART1);
	///////////////////////////////////////////////////////////////
	/* Enable the USART3 interrupt. */
	nvic_enable_irq(NVIC_USART3_IRQ);
	
	/* Setup GPIO pin GPIO_USART3_TX and GPIO_USART3_RX. */
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO11);
	
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		    GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);
	
	/* Setup UART parameters. */
	//usart_set_baudrate(USART3, 38400);
	USART_BRR(USART3) = (u16)((24000000 << 4) / (115200 * 16));
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	
	usart_disable_error_interrupt(USART3);
	usart_enable_rx_interrupt(USART3);
	usart_enable_tx_interrupt(USART3);
	/* Enable USART3 Receive interrupt. */
	//USART_CR1(USART3) |= USART_CR1_RXNEIE;
	
	/* Finally enable the USART. */
	usart_enable(USART3);
}



void usart3_isr(void)
{

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOC, GPIO2);

		/* Retrieve the data from the peripheral. */
		data[data_size] = usart_recv(USART3);
		if(data_size < 50){
		  data_size++;
		} else {
		  data_size = 0;
		}
		/* Enable transmit interrupt so it sends back the data. */
		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}

	if (((USART_CR1(USART3) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_TXE) != 0)) {

		/* Indicate that we are sending out data. */
		// gpio_toggle(GPIOA, GPIO7);

		/* Put data into the transmit register. */
		for(int i=0;i<btBuff_size;i++){
		  usart_send_blocking(USART3, btBuff[i]);
		}
		btBuff_size = 0;
		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(USART3) &= ~USART_CR1_TXEIE;
	}
}
void usart1_isr(void)
{

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		
		
		btBuff[btBuff_size] = usart_recv(USART1);
		if(btBuff[btBuff_size] == 13){
		  /* Indicate that we are sending data. */
		  gpio_toggle(GPIOC, GPIO0);
		  USART_CR1(USART3) |= USART_CR1_TXEIE;
		}
		if(btBuff_size < 1024){
		  btBuff_size++;
		} else {
		  btBuff_size = 0;
		}
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		

		/* Put data into the transmit register. */
		for(int i=0;i<data_size;i++){
		  /* Indicate that we are sending out data. */
		  gpio_toggle(GPIOA, GPIO1);
		  usart_send_blocking(USART1, data[i]);
		}
		data_size = 0;
		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(USART1) &= ~USART_CR1_TXEIE;
	}
}

/*******************************************************************************
* Function Name  : SendChar
* Description    : Send a char by USART3
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int SendChar (int ch)  					/* Write character to Serial Port     */
{      
  usart_send_blocking(USART1, (unsigned char) ch);
  return (ch);
}

/*******************************************************************************
* Function Name  : uart_printf
* Description    : Send a string by USART3
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void uart_printf (char *ptr)
{
	while (*ptr) {
		SendChar (*ptr);
		ptr++;	
	}								
}

void delay_ms(int d){
    int i,j;
    for (j = 0; j < d; j++){
	for (i = 0; i < 4700; i++)	/* Wait a bit. */
	      __asm__("nop");
    }
}

int main(void)
{
  int c=0;
	clock_setup();
	gpio_setup();
	gpio_set(GPIOC, GPIO0 | GPIO1 | GPIO2);
	gpio_clear(GPIOC, GPIO0);
	//enable bt
	gpio_set(GPIOC, GPIO4);
	usart_setup();
	
	
	gpio_clear(GPIOC, GPIO0 | GPIO1 | GPIO2);
	while (1) {
		//rx_value = spi_read(SPI2);
		/* LED on/off */
		if(c%5 == 0){
		  //uart_printf("still here\r\n");
		}
		gpio_toggle(GPIOC, GPIO1);
		delay_ms(1000);
		c++;
	}
}



