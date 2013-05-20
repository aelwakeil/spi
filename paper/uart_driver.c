#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
//#include <libopencm3/stm32/f1/nvic.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/f1/bkp.h>
#include <libopencm3/stm32/crc.h>

#include "paper.h"
#include "uart_driver.h"
#include "spi_driver.h"

uartBuff * t_uart_buff;
u8 config_len = 0;
uart_mode newMode;
u8 lastChar;
char configMode;
u8 configDone;
u8 remainingBytes = 0;
u8 zerosCounter = 0;

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
	//USART_BRR(USART3) = (u16)((24000000 << 4) / (115200 * 16));
	USART_BRR(USART3) = (u16)((24000000 << 4) / (921600 * 16));
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

static void processConfig(void){
  lastChar = 0x00;
  usart_send_blocking(USART3, '$');
  delay_ms(200);
  usart_send_blocking(USART3, '$');
  delay_ms(200);
  usart_send_blocking(USART3, '$');
  delay_ms(250);
  //wait for data
  configDone = 0x00;
  //while(lastChar == 0x00);
  //if(lastChar == 'C'){
    int pos = 0;
    lastChar = 0x00;
    while(pos < config_len){
      if(t_uart_buff->buf[pos] == 0x54){
	pwr_disable_backup_domain_write_protect();
	BKP_DR1 = t_uart_buff->buf[pos+1];
      }
      if(configMode == 'S'){
	if(t_uart_buff->buf[pos]){
	  usart_send_blocking(USART3, t_uart_buff->buf[pos]);
	} else {
	  usart_send_blocking(USART3, '\r');
	  delay_ms(50);
	  usart_send_blocking(USART3, '\n');
	  delay_ms(50);
	  //while(lastChar == 0x00);
	  configMode = ' ';
	  if(configDone == 0x02) {
	      //end of while
	      pos = config_len;
	  }
	}
      }
      if(t_uart_buff->buf[pos] == 0x55){
	uart_printf(USART3, "SP,");
	lastChar = 0x00;
	configDone = 0x01;
	configMode = 'S';
      }
      if(t_uart_buff->buf[pos] == 0x56){
	uart_printf(USART3, "SN,");
	lastChar = 0x00;
	configDone = 0x02;
	configMode = 'S';
      }
      pos++;
    }
  //}
  //and reboot
  delay_ms(250);
  uart_printf(USART3, "---\r");
  delay_ms(250);
  uart_printf(USART3, "R,1\r");
  t_uart_buff->uartMode = IDDLE;
}

void usart3_isr(void)
{
	u8 data = 0x00;
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {
		
		data = usart_recv(USART3);
		gpio_toggle(GPIOC, GPIO0);
		switch(t_uart_buff->uartMode){
		  case DATAMODE:
		    //crc_calculate_block((u32 *) &data, 1);
		    t_uart_buff->buf[t_uart_buff->pointer] = data;
		    t_uart_buff->pointer = t_uart_buff->pointer + 1;
		    t_uart_buff->p = t_uart_buff->p + 1;
		    if(t_uart_buff->pointer + 50000 >= EP_BUFF_SIZE && t_uart_buff->owcounter == 0){
		      t_uart_buff->rdy = 1;
		    }
		    if(t_uart_buff->pointer >= EP_BUFF_SIZE) {
		      t_uart_buff->pointer = 0;
		      t_uart_buff->owcounter = t_uart_buff->owcounter + 1;
		    }
		    break;
		    
		  case CONFIGMODE:
//#ifdef PRODUCTION
		    //todo config packet to buffer and apply changes
// 		    if(config_len == 0){
// 		      config_len = data;
// 		    } else {
// 		      t_uart_buff->buf[t_uart_buff->pointer] = data;
// 		      t_uart_buff->pointer = t_uart_buff->pointer + 1;
// 		      if(config_len >= t_uart_buff->pointer){
// 			config_len = t_uart_buff->pointer;
// 			t_uart_buff->pointer = 0;
// 			t_uart_buff->uartMode = CONFIGBTMODE;
// 			processConfig();
// 		      }
// 		    }
		    t_uart_buff->buf[t_uart_buff->pointer] = data;
		    t_uart_buff->pointer = t_uart_buff->pointer + 1;
		    if(data == 0x00){
		      zerosCounter++;
		      if(zerosCounter >= 3){
			remainingBytes = 4;
		      }
		    }
		    if(remainingBytes < 50) {
		      remainingBytes--;
		    }
		    if(remainingBytes <= 0){
		      config_len = t_uart_buff->pointer - 4;
		      t_uart_buff->pointer = 0;
			t_uart_buff->uartMode = CONFIGBTMODE;
			processConfig();
// 		      crc_reset();
// 		      u32 crc = crc_calculate_block((u32 *) (t_uart_buff->buf), config_len);
// 		      u32 btCrc = (t_uart_buff->buf[config_len+1] << 32) |  (t_uart_buff->buf[config_len+2] << 16) | (t_uart_buff->buf[config_len+3] << 8) | t_uart_buff->buf[config_len+4];
// 		      if(crc != btCrc){
// 			gpio_toggle(GPIOC, GPIO1);
// 		      } else {
// 		      }
		    }
// #else
// 		    usart_send_blocking(USART1, data);
// #endif
		    break;
		  case CONFIGBTMODE:
		    if(lastChar == 0x00 && t_uart_buff->uartMode == CONFIGBTMODE){
		      lastChar = data;
		    }
		    break;
		  case IDDLE:
		  default:
		    //data
		    if(data == 0xAA){
		      //crc_reset();
		      t_uart_buff->start = 1;
		      t_uart_buff->pointer = 0;
		      newMode = DATAMODE;
		    }
		    //config
		    if(data == 0xA9){
		      newMode = CONFIGMODE;
		      t_uart_buff->start = 1;
		      t_uart_buff->pointer = 0;
		      remainingBytes = 50;
		      zerosCounter = 0;
		      //
		      t_uart_buff->uartMode = newMode;
		      t_uart_buff->pointer = 0;
		    }
		    if(t_uart_buff->start > 4){
		      t_uart_buff->uartMode = newMode;
		      t_uart_buff->pointer = 0;
		    } else if(t_uart_buff->start > 0){
		      t_uart_buff->start = t_uart_buff->start + 1;
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
			uart_printf(USART1, "EXIT\n\r");
		      } 
		    }
		    // --- end exit mode ---
		    t_uart_buff->buf[t_uart_buff->pointer] = data;
		    t_uart_buff->pointer = t_uart_buff->pointer + 1;
		    t_uart_buff->p = t_uart_buff->p + 1;
		    if(t_uart_buff->pointer + 10000 >= EP_BUFF_SIZE && t_uart_buff->owcounter == 0){
		      //if(t_uart_buff->pointer + 10000 == EP_BUFF_SIZE) uart_printf(USART1, "Starting rdy = 1\n\r");
		      gpio_toggle(GPIOC, GPIO1);
		      t_uart_buff->rdy = 1;
		    }
		    if(t_uart_buff->pointer >= EP_BUFF_SIZE) {
		      //uart_printf(USART1, "Owerflow\n\r");
		      t_uart_buff->pointer = 0;
		      t_uart_buff->owcounter = t_uart_buff->owcounter + 1;
		    }
		    break;
		    
		  case CONFIGMODE:
		    gpio_toggle(GPIOC, GPIO1);
		    // exit this mode
		    t_uart_buff->buf[t_uart_buff->pointer] = data;
		    t_uart_buff->pointer = t_uart_buff->pointer + 1;
		    if(data == 0x00){
		      zerosCounter++;
		      if(zerosCounter >= 3){
			remainingBytes = 4;
		      }
		    }
		    if(remainingBytes < 50){
		      remainingBytes -= 1;
		    }
		    if(remainingBytes <= 0){
		      config_len = t_uart_buff->pointer - 4;
		      t_uart_buff->pointer = 0;
			t_uart_buff->uartMode = CONFIGBTMODE;
			processConfig();
// 		      crc_reset();
// 		      u32 crc = crc_calculate_block((u32 *) (t_uart_buff->buf), config_len);
// 		      u32 btCrc = (t_uart_buff->buf[config_len+1] << 32) |  (t_uart_buff->buf[config_len+2] << 16) | (t_uart_buff->buf[config_len+3] << 8) | t_uart_buff->buf[config_len+4];
// 		      if(crc != btCrc){
// 			uart_printf(USART1, "CRC FAIL!\n\r");
// 		      } else {
// 		      }
		    }
		      
/*
		      if(data == EXIT_CHAR && t_uart_buff->buf[1] == EXIT_CHAR){
			  t_uart_buff->uartMode = IDDLE;
			  uart_printf(USART1, "EXIT\n\r");
		      }
		      t_uart_buff->buf[1] = data;
		      // --- end exit mode ---
		      //send back
		      if(data == 13){
			usart_send_blocking(USART1, '\n');
		      }
		      usart_send_blocking(USART1, data);
		      //send to BT
		      usart_send_blocking(USART3, data);*/
		    
		    break;
		  case CONFIGBTMODE:
		    lastChar = data;
		    break;
		  case IDDLE:
		  default:
		    if(data == 'D'){
		      newMode = DATAMODE;
		      t_uart_buff->pointer = 0;
		      uart_printf(USART1, "ENTERED DATA MODE\n\n\r");
		    } else if(data == 'C'){
		      t_uart_buff->uartMode = CONFIGMODE;
		      lastChar = 'R';
		      t_uart_buff->start = 0;
		      t_uart_buff->pointer = 0;
		      uart_printf(USART1, "ENTERED CONFIG MODE\n\n\r");
		    } else if(data == 0xA9) {
		      //newMode = CONFIGMODE;
		      t_uart_buff->uartMode = CONFIGMODE;
		      remainingBytes = 50;
		      zerosCounter = 0;
		      t_uart_buff->pointer = 0;
		      //t_uart_buff->start = 1;
		      uart_printf(USART1, "ENTERED CONFIG MODE\n\n\r");
		    } else if(data == 0xAA) {
		      t_uart_buff->start = 1;
		      newMode = DATAMODE;
		    } else {
		      //uart_printf(USART1, "IDDLE mode - do nothing\n\r");
		    }
		    // HEADER 0xAA 0x00 0x01 0x01 ... paper data
		    if(t_uart_buff->start > 1){
		      gpio_toggle(GPIOC, GPIO0);
		      t_uart_buff->uartMode = newMode;
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
* Description    : Send a char by usart
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int SendChar (u32 usart, int ch)  					/* Write character to Serial Port     */
{      
  usart_send_blocking(usart, (unsigned char) ch);
  return (ch);
}

/*******************************************************************************
* Function Name  : uart_printf
* Description    : Send a string by USART
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void uart_printf (u32 usart, char *ptr)
{
	while (*ptr) {
		SendChar (usart, *ptr);
		ptr++;	
	}								
}

void uart_print_int(u32 usart, int value)
{
	s8 i;
	u8 nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send_blocking(usart, '-');
		value = value * -1;
	}

	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}

	for (i = nr_digits; i >= 0; i--) {
		usart_send_blocking(usart, buffer[i]);
	}

	usart_send_blocking(usart, '\r');
}