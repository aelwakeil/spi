#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "uart_driver.h"
#include "spi_driver.h"
#include "paper.h"

void spi_setup(void) {

  /* Configure GPIOs: SS=PB12, SCK=PB13, MISO=PB14 and MOSI=PA15 */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO12 |
					    GPIO13 |
                                            GPIO15 );

  //SPI input
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO14);
  //BUSSY C7
  gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO7);

  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO5 | GPIO6 );
  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(SPI2);

  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/16 of peripheral clock frequency
   * Clock polarity: Idle Low
   * Clock phase: Data valid on 1nd clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_LSBFIRST);

  spi_set_master_mode(SPI2);
  spi_enable_software_slave_management(SPI2);
  spi_enable_ss_output(SPI2);
  spi_set_nss_high(SPI2);

  spi_disable_error_interrupt(SPI2);
  spi_disable_crc(SPI2);
  
  /* Enable SPI2 periph. */
  spi_enable(SPI2);
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

void epClear(unsigned char color){
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
      for (j=0 ; j < 128 ; j++)
      // 1 Line of pixels, 1024/8=128 Bytes
      {
	  spi_send(SPI2,(uint8_t) color); 
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
/*
static u8 switchEndian(u8 data){
  if(htonl(data) == data){
   return (((data>>24)&0xff) | ((data<<8)&0xff0000) | ((data>>8)&0xff00) | ((data<<24)&0xff000000)); 
  }
  return data;
}*/

void EpStartSend(void){
  epTurnOn();
  // Chip Select get low
  delay_ms (50);
  // Delay TCS_SI > 1 ms ; TRESET_CS + TCS_SI ≧ 20ms
  // Send Header Byte
  // Send Header Byte ID = 0x06A0 (for 10.2" EPD)
  
  spi_send(SPI2, (uint8_t) 0x06);
  spi_send(SPI2, (uint8_t) 0xA0);
  delay_ms (5);			// TDELAY1 min 5 ms  
}

void EpStopSend(void){
  ep_cs(1);
  delay_ms (5000);
  // Chip Select get low 
  epTurnOff();
}

void epSendData(uartBuff *buff){
  epTurnOn();
  
  int i,j,k,line;
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
  line = 0;
  for (i=0 ; i < 1280 ; i++)
  //10.2” EPD resolution= 1024 x 1280
  {
      for (j=0 ; j < 128 ; j++)
      // 1 Line of pixels, 1024/8=128 Bytes
      {
	  
	  spi_send(SPI2, ~(buff->buf[k])); 
	  k++;
	  if(k >= EP_BUFF_SIZE){
	    k = 0;
	    //wait for owerflow
	    /*while(buff->owcounter > 0 && buff->pointer < 100 && buff->p < EP_BYTES){
	      //delay_ms(1);
	    }*/
	  }
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
  buff->complete = 1;
}

