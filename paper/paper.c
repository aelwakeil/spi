#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/usart.h>
//project includes
#include "paper_init.h"
#include "spi_driver.h"
#include "uart_driver.h"
#include "paper.h"

#define VERBOSE

uartBuff uart_buff;

int main(void)
{
  int counter = 0;
	//initialize buffer
	uart_buff.p = 0;
	uart_buff.pointer = 0;
	uart_buff.rdy = 0;
	uart_buff.owcounter = 0;
	uart_buff.complete = 0;
	
	clock_setup();
	gpio_setup();
	//gpio_set(GPIOC, GPIO0 | GPIO1 | GPIO2);
	//gpio_clear(GPIOC, GPIO0);
	usart_setup(&uart_buff);
	//gpio_clear(GPIOC, GPIO1);
	spi_setup();
	//gpio_clear(GPIOC, GPIO2);
	
	//gpio_clear(GPIOC, GPIO7);
	/* Blink the LED (PC1) on the board with every transmitted byte. */
	//turn off display, PC12 = BT reset
	gpio_clear(GPIOC, GPIO5 | GPIO6);
	//turn on BT
	gpio_set(GPIOC, GPIO4 | GPIO12);
		
	epClear(0x00);
	while (1) {
		/*if((bbufCounter - bbufReaded) > 0){
		  spi_send(SPI2, bbuff[bbufReaded]);
		  bbufReaded++;
		  if(bbufReaded >= EP_ROUND_BUF ){
		    delay_ms(6);
		    bbufReaded = 0;
		  }
		} else if(bbufCounter > 0) {
		  bbufCounter = 0;
		  bbufReaded = 0;
		}*/
		if(uart_buff.rdy == 1){
		    epSendData(&uart_buff);
		    //uart_buff.start = 0;
		    //uart_buff.owcounter = 0;
		}
		if(uart_buff.complete == 1){
		    for(int c = 0;c<EP_BUFF_SIZE;c++){
		      uart_buff.buf[c] = 0x00;
		    }
		    uart_buff.complete = 0;
		    uart_buff.rdy = 0;
		    uart_buff.pointer = 0;
		}
		/* LED on/off */
		gpio_toggle(GPIOC, GPIO0);
		delay_ms(1000);
		if(counter%5 == 0){
#ifdef VERBOSE
		  uart_printf(",rdy: ");
		  usart_send_blocking(USART1, (unsigned char) uart_buff.rdy+48);
		  uart_printf(", buffer size: ");
 		  usart_send_blocking(USART1, (unsigned char) uart_buff.pointer+48);
		  uart_printf(", OF: ");
 		  usart_send_blocking(USART1, (unsigned char) uart_buff.owcounter+48);
		  uart_printf("\n\r");
#endif
		}
		if(counter == 300){
#ifdef VERBOSE
		  uart_printf("I am going to sleep now");
#endif
		  //10s ... we can sleep e.g.
		  //Turn off LED
		  gpio_set(GPIOC, GPIO0);
		  //turn off BT
		  gpio_clear(GPIOC, GPIO4);
		  //sleep..
		  pwr_set_stop_mode();
		}
		counter++;
	}
}

void delay_ms(int d){
    int i,j;
    for (j = 0; j < d; j++){
	for (i = 0; i < 4900; i++)	/* Wait a bit. */
	      __asm__("nop");
    }
}