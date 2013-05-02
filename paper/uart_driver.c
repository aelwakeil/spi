#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
//#include <libopencm3/stm32/f1/nvic.h>
#include <stdio.h>
#include <errno.h>

#include "paper.h"
#include "uart_driver.h"
#include "spi_driver.h"

uartBuff * t_uart_buff;


void usart_setup(uartBuff *ubuff)
{
  
	t_uart_buff = ubuff;
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
	/* Setup UART parameters. */
	//usart_set_baudrate(USART3, 38400);
	USART_BRR(USART3) = (u16)((24000000 << 4) / (115200 * 16));
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	
	usart_disable_error_interrupt(USART3);
	/* Enable USART3 Receive interrupt. */
	usart_enable_rx_interrupt(USART3);
	usart_enable_tx_interrupt(USART3);
	
	/* Finally enable the USART. */
	usart_enable(USART3);
}



void usart3_isr(void)
{
	u8 data = 0x00;
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {
		
		data = usart_recv(USART3);
		if(data == 0xAA){
		  t_uart_buff->start = 1;
		  t_uart_buff->pointer = 0;
		  //t_uart_buff->rdy = 1;
		  //EpStartSend();
		}
		if(t_uart_buff->start >= 3){
		  gpio_toggle(GPIOC, GPIO0);
		  t_uart_buff->buf[t_uart_buff->pointer] = data;
		  t_uart_buff->pointer = t_uart_buff->pointer + 1;
		  if(t_uart_buff->pointer + 1300 >= EP_BUFF_SIZE){
		    t_uart_buff->rdy = 1;
		    //USART_CR1(USART3) |= USART_CR1_TXEIE;
		  }
		  if(t_uart_buff->pointer >= EP_BUFF_SIZE) {
		    //t_uart_buff->rdy = 1;
		    t_uart_buff->pointer = 0;
		    t_uart_buff->owcounter = t_uart_buff->owcounter + 1;
		    //USART_CR1(USART1) |= USART_CR1_TXEIE;
		  }
		  /*t_uart_buff->buf[t_uart_buff->pointer] = data;
		  t_uart_buff->pointer = t_uart_buff->pointer + 1;
		  if(t_uart_buff->pointer >= EP_BUFF_SIZE) {
		    gpio_toggle(GPIOC, GPIO0 | GPIO1 | GPIO2);
		    t_uart_buff->rdy = 1;
		    t_uart_buff->pointer = 0;
		    t_uart_buff->owcounter = t_uart_buff->owcounter + 1;
		    USART_CR1(USART1) |= USART_CR1_TXEIE;
		  }*/
		} else if(t_uart_buff->start >= 0){
		  t_uart_buff->start = t_uart_buff->start + 1;
		}
	}

	if (((USART_CR1(USART3) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_TXE) != 0)) {

		/* Indicate that we are sending out data. */
		// gpio_toggle(GPIOA, GPIO7);
		//do something ---> send to BT
		/*if(t_uart_buff->start >= 3){
		  epSendData(t_uart_buff);
		  t_uart_buff->rdy = 0;
		}*/
		
		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(USART3) &= ~USART_CR1_TXEIE;
	}
}
void usart1_isr(void)
{
	u8 data = 0x00;
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
		/* Retrieve the data from the peripheral. */
		
		gpio_toggle(GPIOC, GPIO1);
		data = usart_recv(USART1);

		t_uart_buff->buf[t_uart_buff->pointer] = data;
		t_uart_buff->pointer = t_uart_buff->pointer + 1;
		if(t_uart_buff->pointer + 1300 >= EP_BUFF_SIZE){
		  gpio_toggle(GPIOC, GPIO1);
		  t_uart_buff->rdy = 1;
		  t_uart_buff->start = 4;
		  //USART_CR1(USART1) |= USART_CR1_TXEIE;
		}
		if(t_uart_buff->pointer >= EP_BUFF_SIZE) {
		  //t_uart_buff->rdy = 1;
		  t_uart_buff->pointer = 0;
		  t_uart_buff->owcounter = t_uart_buff->owcounter + 1;
		  //USART_CR1(USART1) |= USART_CR1_TXEIE;
		}
		
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {
		/*epSendData(t_uart_buff);
		t_uart_buff->rdy = 0;
		t_uart_buff->pointer = 0;
		*/
		/*
		uart_printf("\n\rbuffer full, I am going to paint now\n\r");
		usart_send_blocking(USART3, 'O');
		usart_send_blocking(USART3, 'K');
		usart_send_blocking(USART3, '\n');
		usart_send_blocking(USART3, '\r');*/
		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(USART1) &= ~USART_CR1_TXEIE;
	}
}



/*******************************************************************************
* Function Name  : SendChar
* Description    : Send a char by USART1
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
* Description    : Send a string by USART1
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