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
	/* Enable the USART3 interrupt. */
	nvic_enable_irq(NVIC_USART3_IRQ);
#ifndef PRODUCTION
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
#endif
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
		switch(t_uart_buff->uartMode){
		  case DATAMODE:
		    gpio_toggle(GPIOC, GPIO0);
		    t_uart_buff->buf[t_uart_buff->pointer] = data;
		    t_uart_buff->pointer = t_uart_buff->pointer + 1;
		    t_uart_buff->p = t_uart_buff->p + 1;
		    if(t_uart_buff->pointer + 10000 >= EP_BUFF_SIZE && t_uart_buff->owcounter == 0){
		      t_uart_buff->rdy = 1;
		    }
		    if(t_uart_buff->pointer >= EP_BUFF_SIZE) {
		      t_uart_buff->pointer = 0;
		      t_uart_buff->owcounter = t_uart_buff->owcounter + 1;
		    }
		    break;
		    
		  case CONFIGMODE:
		    usart_send_blocking(USART1, data);
		    break;
		    
		  case IDDLE:
		  default:
		    //data
		    if(data == 0xAA){
		      t_uart_buff->start = 1;
		      t_uart_buff->pointer = 0;
		    }
		    if(t_uart_buff->start >= 3){
		      t_uart_buff->uartMode = DATAMODE;
		      t_uart_buff->pointer = 0;
		    } else if(t_uart_buff->start > 0){
		      t_uart_buff->start = t_uart_buff->start + 1;
		    }
		    //config
		    if(data == 0xCC){
		      t_uart_buff->uartMode = CONFIGMODE;
		    }
		}

		
	}

	if (((USART_CR1(USART3) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_TXE) != 0)) {
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
		switch(t_uart_buff->uartMode){
		  case DATAMODE:
		    // exit this mode
		    if(data == EXIT_CHAR && t_uart_buff->pointer > 1){
		      if(t_uart_buff->buf[t_uart_buff->pointer - 1] == EXIT_CHAR){
			t_uart_buff->uartMode = IDDLE;
			uart_printf("EXIT\n\r");
		      } 
		    }
		    // --- end exit mode ---
		    t_uart_buff->buf[t_uart_buff->pointer] = data;
		    t_uart_buff->pointer = t_uart_buff->pointer + 1;
		    t_uart_buff->p = t_uart_buff->p + 1;
		    if(t_uart_buff->pointer + 15000 >= EP_BUFF_SIZE && t_uart_buff->owcounter == 0){
		      if(t_uart_buff->pointer + 15000 == EP_BUFF_SIZE) uart_printf("Starting rdy = 1\n\r");
		      gpio_toggle(GPIOC, GPIO1);
		      t_uart_buff->rdy = 1;
		    }
		    if(t_uart_buff->pointer >= EP_BUFF_SIZE) {
		      uart_printf("Owerflow\n\r");
		      t_uart_buff->pointer = 0;
		      t_uart_buff->owcounter = t_uart_buff->owcounter + 1;
		    }
		    break;
		    
		  case CONFIGMODE:
		    // exit this mode
		    if(data == EXIT_CHAR && t_uart_buff->buf[1] == EXIT_CHAR){
			t_uart_buff->uartMode = IDDLE;
			uart_printf("EXIT\n\r");
		    }
		    t_uart_buff->buf[1] = data;
		    // --- end exit mode ---
		    //send back
		    if(data == 13){
		      usart_send_blocking(USART1, '\n');
		    }
		    usart_send_blocking(USART1, data);
		    //send to BT
		    usart_send_blocking(USART3, data);
		    break;
		    
		  case IDDLE:
		  default:
		    if(data == 'D'){
		      t_uart_buff->uartMode = DATAMODE;
		      t_uart_buff->pointer = 0;
		      uart_printf("ENTERED DATA MODE\n\n\r");
		    } else if(data == 'C'){
		      t_uart_buff->uartMode = CONFIGMODE;
		      t_uart_buff->start = 0;
		      t_uart_buff->pointer = 0;
		      uart_printf("ENTERED CONFIG MODE\n\n\r");
		    } else if(data == 0xAA) {
		      t_uart_buff->start = 1;
		    } else {
		      uart_printf("IDDLE mode - do nothing\n\r");
		    }
		    if(t_uart_buff->start >= 3){
		      t_uart_buff->uartMode = DATAMODE;
		      t_uart_buff->pointer = 0;
		    } else if(t_uart_buff->start > 0){
		      t_uart_buff->start = t_uart_buff->start + 1;
		    }
		}

		
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {
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