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
#include <libopencm3/stm32/f1/nvic.h>

#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/spi.h>
#include <stdio.h>
#include <errno.h>

void delay_ms(int d);
int SendChar (int ch);
void uart_printf (char *ptr);

static unsigned int lfsr113_Bits (void)
{
   static unsigned int z1 = 12345, z2 = 12345, z3 = 12345, z4 = 12345;
   unsigned int b;
   b  = ((z1 << 6) ^ z1) >> 13;
   z1 = ((z1 & 4294967294U) << 18) ^ b;
   b  = ((z2 << 2) ^ z2) >> 27; 
   z2 = ((z2 & 4294967288U) << 2) ^ b;
   b  = ((z3 << 13) ^ z3) >> 21;
   z3 = ((z3 & 4294967280U) << 7) ^ b;
   b  = ((z4 << 3) ^ z4) >> 12;
   z4 = ((z4 & 4294967168U) << 13) ^ b;
   return (z1 ^ z2 ^ z3 ^ z4);
}

static void clock_setup(void)
{
	//rcc_clock_setup_in_hse_12mhz_out_72mhz();
	rcc_clock_setup_in_hse_8mhz_out_24mhz();
	/* Enable GPIOA, GPIOB, GPIOC clock. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
				    RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);

	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
// 	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
// 	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);

	/* Enable SPI2 Periph and gpio clocks */
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
				    RCC_APB1ENR_SPI2EN);
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
	USART_BRR(USART1) = (u16)((24000000 << 4) / (155200 * 16));
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
	nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);

	nvic_set_priority(NVIC_DMA1_CHANNEL6_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL6_IRQ);
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
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
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
  spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_4, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_0, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  //spi_set_bidirectional_transmit_only_mode(SPI2);
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
      spi_set_nss_high(SPI2);
      //gpio_set(GPIOB, GPIO12);
    } else {
      spi_set_nss_low(SPI2);
      //gpio_clear(GPIOB, GPIO12);
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
    gpio_set(GPIOC, GPIO5); //Vcc
    delay_ms(5);
    ep_cs(1);		// Chip Select get high
    delay_ms(11);	// TVcc_on > 10 ms PE

    //Reset Display TCON
    ep_on(1);		// RESET get high
    delay_ms (19);	// Delay 19 ms
    ep_cs(0);		// Chip Select get high
}

static void epClear(uint8_t color){
  epTurnOn();
  
  int i,j;
  // Chip Select get low
  delay_ms (50);
  // Delay TCS_SI > 1 ms ; TRESET_CS + TCS_SI ≧ 20ms
  // Send Header Byte
  // Send Header Byte ID = 0x06A0 (for 10.2" EPD)
  
  spi_send(SPI2, (uint8_t) 0x06);
  spi_send(SPI2, (uint8_t) 0xA0);
  delay_ms (5);			// TDELAY1 min 5 ms
  // Transmit Display Pattern
  for (i=0 ; i < 1280 ; i++)
  //10.2” EPD resolution= 1024 x 1280
  {
      for (j=0 ; j < 64 ; j++)
      // 1 Line of pixels, 1024/8/2=64 Bytes
      {
	  spi_send(SPI2, color); 
	  spi_send(SPI2, color);// // Byte2, Byte1, “Black” “Black”  for example example.
	  //Tdelay2 min 0 ms
	  //delay_ms (1); 
      }
      //Tdelay3 min 5ms
      delay_ms(6);
  }
    //wait for BUSSY
  //while((GPIOA_IDR & GPIO7) == 0 );
  ep_cs(1);
  delay_ms (5000);
  // Chip Select get low 
  epTurnOff();
}

static uint8_t blackOrWhite(int x, int y){
    if(x > 28 && x < 100 && y > 280 && y < 1001) {
      return 0xFF;
    }
    return 0x00;
}

static void epSendData(void){
  epTurnOn();
  
  int i,j,k,l;
  uint8_t color;
  if(lfsr113_Bits() > 0.5){
    color = 0xFF;
  } else {
    color = 0x00;
  }
  // Chip Select get low
  delay_ms (50);
  // Delay TCS_SI > 1 ms ; TRESET_CS + TCS_SI ≧ 20ms
  // Send Header Byte
  // Send Header Byte ID = 0x06A0 (for 10.2" EPD)
  
  spi_send(SPI2, (uint8_t) 0x06);
  spi_send(SPI2, (uint8_t) 0xA0);
  delay_ms (5);			// TDELAY1 min 5 ms
  // Transmit Display Pattern
  k = 0;
  l = 0;
  for (i=0 ; i < 1280 ; i++)
  //10.2” EPD resolution= 1024 x 1280
  {
      k = 0;
      for (j=0 ; j < 128 ; j++)
      // 1 Line of pixels, 1024/8=128 Bytes
      {
	  color = blackOrWhite(k,l);
	  spi_send(SPI2, color); 
	  //Tdelay2 min 0 ms
	  //delay_ms (1); 
// 	  if(k > 16 && k < 49 && l > 280 && l < 1001) {
// 	    /*if(k > 17 && k < 48 && l > 290 && l < 995){
// 	      if(k == 33 && l > 350 && l < 936 || l > 350 && l < 356 || l > 930 && l < 936){
// 		color = 0xFF;
// 	      }else{
// 		color = 0x00;
// 	      }
// 	    }else{
// 	      color = 0xFF;
// 	    }*/
// 	    
// 	  } else {
// 	    color = 0x00;
// 	  }
	  k++;
      }
      l++;
      //Tdelay3 min 5ms
      delay_ms(6);
  }
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
	usart_setup();
	gpio_clear(GPIOC, GPIO1);
	spi_setup();
	gpio_clear(GPIOC, GPIO2);
	
	//gpio_clear(GPIOC, GPIO7);
	/* Blink the LED (PC1) on the board with every transmitted byte. */
	//turn off display
	gpio_clear(GPIOC, GPIO5 | GPIO6);
	
	//epClear(0x00);
	//delay_ms(1000);
	//epClear(0xFF);
	epSendData();
	while (1) {
		/* LED on/off */
		gpio_toggle(GPIOC, GPIO1);
		//uart_printf("Test mode\r\n");
		delay_ms(1000);
	}
}

void delay_ms(int d){
    int i,j;
    for (j = 0; j < d; j++){
	for (i = 0; i < 4900; i++)	/* Wait a bit. */
	      __asm__("nop");
    }
}