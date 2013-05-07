#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/usart.h>
//project includes
#include "paper_init.h"
#include "spi_driver.h"
#include "uart_driver.h"
#include "paper.h"

#define VERBOSE
//number of seconds to sleep
#define SLEEP_TIME 300

//#DEFINE PRODUCTION

uartBuff uart_buff;

static void init_ep_buff(void){
	uart_buff.p = 0;
	uart_buff.pointer = 0;
	uart_buff.rdy = 0;
	uart_buff.owcounter = 0;
	uart_buff.complete = 0;
	uart_buff.uartMode = IDDLE;
}

int main(void)
{
  int counter = 0;
	//initialize buffer
	init_ep_buff();
	clock_setup();
	gpio_setup();
	gpio_set(GPIOC, GPIO0 | GPIO1 | GPIO2);
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
		
	//epClear(0x00);
	while (1) {
		if(uart_buff.rdy == 1){
		    uart_printf("Painting started!\n\r");
		    epSendData(&uart_buff);
		    //uart_buff.start = 0;
		    //uart_buff.owcounter = 0;
		}
		if(uart_buff.complete == 1){
		    //erase buffer
		    for(int c = 0;c<EP_BUFF_SIZE;c++){
		      uart_buff.buf[c] = 0x00;
		    }
		    init_ep_buff();
		}
		/* LED on/off */
		gpio_toggle(GPIOC, GPIO0);
		//delay_ms(1000);
		if(uart_buff.uartMode != IDDLE){
		  counter = 0;
		}
		if(counter%5 == 0 && uart_buff.uartMode == IDDLE){
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
		if(counter == SLEEP_TIME && 0){
#ifdef VERBOSE
		  uart_printf("I am going to sleep now");
#endif
		  //10s ... we can sleep e.g.
		  //Turn off LED
		  gpio_set(GPIOC, GPIO0);
		  //turn off BT
		  gpio_clear(GPIOC, GPIO4);
		  //sleep..
		  pwr_set_standby_mode();
		  pwr_set_stop_mode();
		  //while(1);
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