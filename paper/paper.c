#include <libopencm3/stm32/gpio.h>

#include <libopencm3/stm32/usart.h>
//project includes
#include "paper_init.h"
#include "spi_driver.h"
#include "uart_driver.h"
#include "paper.h"



uartBuff uart_buff;

int main(void)
{
  int counter = 0;
	//initialize buffer
	uart_buff.pointer = 0;
	uart_buff.rdy = 0;
	uart_buff.owcounter = 0;
	
	clock_setup();
	gpio_setup();
	gpio_set(GPIOC, GPIO0 | GPIO1 | GPIO2);
	delay_ms(300);
	gpio_clear(GPIOC, GPIO0 | GPIO1 | GPIO2);
	delay_ms(300);
	gpio_set(GPIOC, GPIO0 | GPIO1 | GPIO2);
	gpio_clear(GPIOC, GPIO0);
	usart_setup(&uart_buff);
	gpio_clear(GPIOC, GPIO1);
	spi_setup();
	gpio_clear(GPIOC, GPIO2);
	
	//gpio_clear(GPIOC, GPIO7);
	/* Blink the LED (PC1) on the board with every transmitted byte. */
	//turn off display
	//gpio_clear(GPIOC, GPIO5 | GPIO6);
	gpio_set(GPIOC, GPIO5);
	//turn on BT
	gpio_set(GPIOC, GPIO4);
	delay_ms(10);
	gpio_clear(GPIOC, GPIO4);
	gpio_set(GPIOC, GPIO4);
	delay_ms(10);
	gpio_clear(GPIOC, GPIO4);
	gpio_set(GPIOC, GPIO4);
		
	//epClear(0x00);
	while (1) {
		if(uart_buff.rdy == 1){
		    epSendData(&uart_buff);
		    uart_buff.rdy = 0;
		    uart_buff.pointer = 0;
		    //uart_buff.owcounter = 0;
		}
		/* LED on/off */
		gpio_toggle(GPIOC, GPIO1);
		//uart_printf("Test mode\r\n");
		delay_ms(1000);
		if(counter%5 == 0){
		  usart_wait_send_ready(USART1);
		  uart_printf("buf: ");
		  for (int c=0;c<uart_buff.pointer;c++){
		    usart_send_blocking(USART1, (unsigned char) uart_buff.buf[c]);
		  }
		  uart_printf(",rdy: ");
		  usart_send_blocking(USART1, (unsigned char) uart_buff.rdy+48);
		  uart_printf(", buffer size: ");
 		  usart_send_blocking(USART1, (unsigned char) uart_buff.pointer+48);
		  uart_printf(", OF: ");
 		  usart_send_blocking(USART1, (unsigned char) uart_buff.owcounter+48);
		  uart_printf("\n\r");
		}
		if(counter == 10){
		  //10s ... we can sleep e.g.
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