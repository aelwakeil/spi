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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
#include <stdio.h>
#include <errno.h>

void delay_ms(int d);
int SendChar (int ch);
void uart_printf (char *ptr);

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_24mhz();
	/* Enable GPIOA, GPIOB, GPIOC clock. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
				    RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);

	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);

	/* Enable SPI1 Periph and gpio clocks */
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
				    RCC_APB2ENR_SPI1EN);
}

static void usart_setup(void)
{
	/* Setup GPIO pin GPIO_USART1_TX and GPIO_USART1_RX. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	/*gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	*/
	/* Setup UART parameters. */
	//usart_set_baudrate(USART1, 38400);
	USART_BRR(USART1) = (u16)((24000000 << 4) / (38400 * 16));
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void gpio_setup(void)
{
	/* Set GPIO1 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1 | GPIO2);
}

static void spi_setup(void) {

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
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO5 |
					    GPIO6 );
  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(SPI1);

  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/64 of peripheral clock frequency
   * Clock polarity: Idle High
   * Clock phase: Data valid on 2nd clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);

  /* Enable SPI1 periph. */
  spi_enable(SPI1);
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
      gpio_set(GPIOB, GPIO12);
    } else {
      gpio_clear(GPIOB, GPIO12);
    }
}

static void ep_on(int val){
    //BP12 
    if(val == 1){
      gpio_set(GPIOC, GPIO6);
    } else {
      gpio_clear(GPIOC, GPIO6);
    }
}
static void epTurnOff(void){
    delay_ms(500);	// TVcc_on > 10 ms PE
    //turn on EP
    gpio_clear(GPIOC, GPIO5);
}
static void epTurnOn(void){
  
    ep_cs(0);
    ep_on(0);		// V Chip RESET Set E T Select SETUP get = low get 40% low x TCLK ~60% x TCLK
    delay_ms(10);	// TVcc_on > 10 ms PE

    //Reset Display TCON
    ep_cs(1);		// Chip Select get high
    ep_on(1);		// RESET get high
    delay_ms (5);	// Delay 5 ms
    ep_on(0);		// RESET get low
    delay_ms (5);	// Delay TRESET > 5 ms
    ep_on(1);		// Reset high
    delay_ms(19);	// TRESET_CS > 19 ms
    gpio_set(GPIOC, GPIO5);
}
static void epSendData(void){
  epTurnOn();
  
  int i,j;
  ep_cs(0);
  // Chip Select get low
  delay_ms (1);
  // Delay TCS_SI > 1 ms ; TRESET_CS + TCS_SI ≧ 20ms
  // Send Header Byte
  spi_send(SPI1,(uint8_t) 0x06);
  spi_send(SPI1,(uint8_t) 0xA0);
  // Send Header Byte ID = 0x06A0 (for 10.2" EPD)
  delay_ms (120);
  // 120 ms ≦ TDELAY1 ≦ 150 ms
  // Transmit Display Pattern
  for (i=0 ; i < 1280 ; i++)
  //10.2” EPD resolution= 1024 x 1280
  {
      for (j=0 ; j < 64 ; j++)
      // 1 Line of pixels, 1024/8/2=16.5 Bytes
      {
	spi_send(SPI1,(uint8_t) 0xFF); 
	spi_send(SPI1,(uint8_t) 0xFF);// // Byte2, Byte1, “Black” “Black”  for example example.
	delay_ms (1); 
      }
      
      spi_send(SPI1,(uint8_t) 0xFF); // 
      spi_send (SPI1,(uint8_t) 0x00);
      delay_ms(1);
  }
    //wait for BUSSY
  while((GPIOA_IDR & GPIO7) == 0 );
  ep_cs(1);
  delay_ms (2500);
  ep_cs(0);
  // Chip Select get low 
  epTurnOff();
}


int main(void)
{
	clock_setup();
	gpio_setup();
	gpio_clear(GPIOC, GPIO0);
	usart_setup();
	gpio_clear(GPIOC, GPIO1);
	spi_setup();
	gpio_clear(GPIOC, GPIO2);
	
	//gpio_clear(GPIOC, GPIO7);
	/* Blink the LED (PC1) on the board with every transmitted byte. */
	
	while (1) {
		epSendData();
		/* LED on/off */
		gpio_toggle(GPIOC, GPIO1);
		uart_printf("Test mode\r\n");
		delay_ms(1000);
	}

	return 0;
}

void delay_ms(int d){
    int i,j;
    for (j = 0; j < d; j++){
	for (i = 0; i < 4100; i++)	/* Wait a bit. */
	      __asm__("nop");
    }
}
