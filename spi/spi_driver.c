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
//#include <libopencm3/stm32/rcc.h>
//#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
//#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <stdio.h>
#include <errno.h>

#define BUFFER_SIZE 1024

struct uartBuff {
	u8 *data;
	s32 size;
	u32 begin;
	u32 end;
};

#define BUFF_SIZE(BUFF)  ((BUFF)->size - 1)
#define BUFF_DATA(BUFF)  (BUFF)->data
#define BUFF_EMPTY(BUFF) ((BUFF)->begin == (BUFF)->end)
struct uartBuff output_buff;
u8 output_uart_buffer[BUFFER_SIZE];

int dataReady = 0;

void delay_ms(int d);
int SendChar (int ch);
void uart_printf (char *ptr);

static void buff_init(struct uartBuff *buff, u8 *buf, s32 size)
{
	buff->data = buf;
	buff->size = size;
	buff->begin = 0;
	buff->end = 0;
}


static s32 buff_write_ch(struct uartBuff *buff, u8 ch)
{
	if (((buff->end + 1) % buff->size) != buff->begin) {
		buff->data[buff->end++] = ch;
		buff->end %= buff->size;
		return (u32)ch;
	}

	return -1;
}
static s32 buff_read_ch(struct uartBuff *buff, u8 *ch)
{
	s32 ret = -1;

	if (buff->begin != buff->end) {
		ret = buff->data[buff->begin++];
		buff->begin %= buff->size;
		if (ch)
			*ch = ret;
	}

	return ret;
}

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_24mhz();
	/* Enable GPIOA, GPIOB, GPIOC clock. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
				    RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);

	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN |
				    RCC_APB2ENR_AFIOEN);

	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);

	/* Enable SPI2 Periph and gpio clocks */
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
				    RCC_APB1ENR_SPI2EN);
}

static void usart_setup(void)
{
	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);

	buff_init(&output_buff, output_uart_buffer, BUFFER_SIZE);
	
	/* Setup GPIO pin GPIO_USART1_TX and GPIO_USART1_RX. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	
	/* Setup UART parameters. */
	//usart_set_baudrate(USART1, 38400);
	USART_BRR(USART1) = (u16)((24000000 << 4) / (38400 * 16));
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;
	
	/* Finally enable the USART. */
	usart_enable(USART1);
}



void usart1_isr(void)
{
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOC, GPIO2);

		/* Retrieve the data from the peripheral. */
		buff_write_ch(&output_buff, usart_recv(USART1));

		/* Enable transmit interrupt so it sends back the data. */
		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		s32 data;

		data = buff_read_ch(&output_buff, NULL);

		if (data == -1) {
			/* Disable the TXE interrupt, it's no longer needed. */
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
			dataReady = 1;
		} else {
			/* Put data into the transmit register. */
			usart_send(USART1, data);
		}
	}
}

static void gpio_setup(void)
{
	/* Set GPIO1 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1 | GPIO2);
}

static void spi_setup(void) {

  /* Configure GPIOs: SS=PA4, SCK=PA5, MISO=PA6 and MOSI=PA7 */
  /*gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4 |
					    GPIO5 |
                                            GPIO7 );

  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
          GPIO6);
  */
  /* Configure GPIOs: SS=PB12, SCK=PB13, MISO=PB14 and MOSI=PA15 */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO12 |
					    GPIO13 |
                                            GPIO15 );

  //SPI input
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO14);
  //BUSSY C7
  gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO7);
  //C6 = reset, C7 = bussy, C5 = EP_on = 0
  /*gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 |
					    GPIO6 |
					    GPIO7 );
  */
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO5 |
					    GPIO6 );
  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(SPI2);

  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/64 of peripheral clock frequency
   * Clock polarity: Idle High
   * Clock phase: Data valid on 2nd clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
//  spi_init_master(SPI2, 1000000, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
//                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT,
//                   SPI_CR1_LSBFIRST);
  spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_set_master_mode(SPI2);
  spi_enable_software_slave_management(SPI2);
  spi_enable_ss_output(SPI2);
  spi_set_nss_high(SPI2);

  spi_disable_error_interrupt(SPI2);
  spi_disable_crc(SPI2);
  /* Enable SPI2 periph. */
  spi_enable(SPI2);
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


static void ep_cs(int val){
    //BP12 
    if(val == 1){
      //spi_set_nss_high(SPI2);
      gpio_set(GPIOB, GPIO12);
    } else {
      //spi_set_nss_low(SPI2);
      gpio_clear(GPIOB, GPIO12);
    }
}

static void ep_on(int val){
  //Vcc
    //BP12 
    if(val == 1){
      gpio_set(GPIOC, GPIO6);
    } else {
      gpio_clear(GPIOC, GPIO6);
    }
}
static void epTurnOff(void){
    delay_ms(500);	// TVcc_on > 10 ms PE
    //turn off EP
    ep_on(0);		// V Chip RESET Set E T Select SETUP get = low get 40% low x TCLK ~60% x TCLK
    gpio_clear(GPIOC, GPIO5);
}
static void epTurnOn(void){
    ep_cs(0);
    ep_on(0);		// V Chip RESET Set E T Select SETUP get = low get 40% low x TCLK ~60% x TCLK
    gpio_set(GPIOC, GPIO5);
    delay_ms(10);

    ep_cs(1);		// Chip Select get high
    delay_ms(35);	// TVcc_on > 10 ms PE

    //Reset Display TCON
    ep_on(1);		// RESET get high
    delay_ms (39);	// Delay 19 ms
    ep_cs(0);		// Chip Select get high
}
static void epSendData(void){
  epTurnOn();
  s32 data;

  int i,j,bufPointer;
  // Chip Select get low
  delay_ms (30);
  // Delay TCS_SI > 1 ms ; TRESET_CS + TCS_SI ≧ 20ms
  // Send Header Byte
  // Send Header Byte ID = 0x06A0 (for 10.2" EPD)
  
  spi_send(SPI2, (uint8_t) 0x06);
  spi_send(SPI2, (uint8_t) 0xA0);
  delay_ms (20);			// TDELAY1 min 5 ms
  // Transmit Display Pattern
  
  for (i=0 ; i < 1280 ; i++)
  //10.2” EPD resolution= 1024 x 1280
  {
      bufPointer = 0;
      for (j=0 ; j < 64 ; j++)
      // 1 Line of pixels, 1024/8/2=64 Bytes
      {
	  spi_send(SPI2, 0x00); 
	  spi_send(SPI2, 0x00); 
	  /*
	  data = buff_read_ch(&output_buff, NULL);
	  spi_send(SPI2, data > 'A' ? 0xFF : 0x00); 
	  bufPointer++;
	  
	  data = buff_read_ch(&output_buff, NULL);
	  spi_send(SPI2, data > 'A' ? 0xFF : 0x00); 
	  bufPointer++;*/
	  //Tdelay2 min 0 ms
	  //delay_ms (1); 
	  
      }
      //Tdelay3 min 5ms
      delay_ms(6);
  }
  gpio_toggle(GPIOC, GPIO0);
    //wait for BUSSY
  //while((GPIOA_IDR & GPIO7) == 0 );
  ep_cs(1);
  delay_ms (5000);
  // Chip Select get low 
  epTurnOff();
}


int main(void)
{
	clock_setup();
	gpio_setup();
	gpio_set(GPIOC, GPIO0 | GPIO1 | GPIO2);
	gpio_clear(GPIOC, GPIO0);
	//usart_setup();
	gpio_clear(GPIOC, GPIO1);
	spi_setup();
	gpio_clear(GPIOC, GPIO2);
	
	/* Blink the LED (PC1) on the board with every transmitted byte. */
	//turn off display
	gpio_clear(GPIOC, GPIO5 | GPIO6);
	
	epSendData();
	gpio_clear(GPIOC, GPIO0 | GPIO1 | GPIO2);
	while (1) {
		gpio_set(GPIOC, GPIO0);
		if(dataReady == 1){
		    gpio_clear(GPIOC, GPIO0);
		    dataReady = 0;
		    epSendData();
		}
		//rx_value = spi_read(SPI2);
		/* LED on/off */
		gpio_toggle(GPIOC, GPIO1);
		//uart_printf("Test mode\r\n");
		delay_ms(1000);
	}
}


void delay_ms(int d){
    int i,j;
    for (j = 0; j < d; j++){
	for (i = 0; i < 4700; i++)	/* Wait a bit. */
	      __asm__("nop");
    }
}
