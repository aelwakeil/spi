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
#include <libopencm3/stm32/f1/dma.h>
#include <stdio.h>
#include <errno.h>


void delay_ms(int d);
int SendChar (int ch);
void uart_printf (char *ptr);

u8 data[50];
int data_size = 0;
int data_pointer = 0;

u8 btBuff[1024];
int btBuff_size = 0;
int btBuff_pointer = 0;

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
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1 | GPIO2 | GPIO4 | GPIO12);
}

static void usart_setup(void)
{
  	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);
	/* Enable the USART3 interrupt. */
	nvic_enable_irq(NVIC_USART3_IRQ);
	
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
	//tx
	//nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
	//nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
	//Rx
	//nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, 0);
	//nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);

	///////////////////////////////////////////////////////////////

	
	/* Setup GPIO pin GPIO_USART3_TX and GPIO_USART3_RX. */
	AFIO_MAPR |= AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP;
	/* RESET */
	gpio_set(GPIOC, GPIO12);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);

	/* TX */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);
	/* RX */
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO11);

	/* CTS */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN, GPIO8);
	/* RTS */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);

	/*AFIO_MAPR |= AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP;
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_PR_TX);
	
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART3_PR_RX);
	*/	      
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



static void dma_write(char *data, int size)
{
	/*
	 * Using channel 4 for USART1_TX
	 */

	/* Reset DMA channel*/
	dma_channel_reset(DMA1, DMA_CHANNEL4);

	dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (u32)&USART1_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL4, (u32)data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL4, size);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_VERY_HIGH);

	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);

	dma_enable_channel(DMA1, DMA_CHANNEL4);

        usart_enable_tx_dma(USART1);
}

volatile int transfered = 0;

void dma1_channel5_isr(void)
{
	if ((DMA1_ISR &DMA_ISR_TCIF7) != 0) {
		DMA1_IFCR |= DMA_IFCR_CTCIF5;

		transfered = 1;
	}

	dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

	usart_disable_tx_dma(USART1);

	dma_disable_channel(DMA1, DMA_CHANNEL4);
}

static void dma_read(char *data, int size)
{
	/*
	 * Using channel 5 for USART1_RX
	 */

	/* Reset DMA channel*/
	dma_channel_reset(DMA1, DMA_CHANNEL5);

	dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (u32)&USART1_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL5, (u32)data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL5, size);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL5);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_HIGH);

	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);

	dma_enable_channel(DMA1, DMA_CHANNEL5);

        usart_enable_rx_dma(USART1);
}

volatile int received = 0;

void dma1_channel4_isr(void)
{
	if ((DMA1_ISR &DMA_ISR_TCIF4) != 0) {
		DMA1_IFCR |= DMA_IFCR_CTCIF5;

		received = 1;
	}

	dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

	usart_disable_rx_dma(USART1);

	dma_disable_channel(DMA1, DMA_CHANNEL4);
}



void usart3_isr(void)
{
    u8 newdata;
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOC, GPIO2);
	
		/* Retrieve the data from the peripheral. */
		newdata = usart_recv(USART3);
		usart_send_blocking(USART1, newdata);
		/*newdata = data[data_size];
		 * if(data_size < 50){
		  data_size++;
		} else {
		  data_size = 0;
		}*/
		/* Enable transmit interrupt so it sends back the data. */
		//USART_CR1(USART1) |= USART_CR1_TXEIE;
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

      u8 newdata;
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		
		  /* Indicate that we are sending data. */
		  gpio_toggle(GPIOC, GPIO0);
		newdata = usart_recv(USART1);
		usart_send_blocking(USART1, newdata);
		if(newdata == 13){
		  usart_send_blocking(USART1, '\n');
		  USART_CR1(USART3) |= USART_CR1_TXEIE;
		  if(btBuff_size > 0 && btBuff[btBuff_size-1] != '$'){
		    btBuff[btBuff_size] = newdata;
		    btBuff_size++;
		  }
		} else {
		  btBuff[btBuff_size] = newdata;
		  if(btBuff_size < 1024){
		    btBuff_size++;
		  } else {
		    btBuff_size = 0;
		  }
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
	gpio_set(GPIOC, GPIO4 | GPIO12);
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



