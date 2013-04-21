
#include "spi.c"




int main(void)
{
  //int counter = 5;
  //u16 rx_value = 0x42;
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
	gpio_set(GPIOC, GPIO0 | GPIO1 | GPIO2);
	//epClear();
	//epSendData();
	while (1) {
		/* printf the value that SPI should send */
		/* blocking send of the byte out SPI1 */
		//epTurnOn();
		//spi_send(SPI2, (uint8_t) 0x80);
		/* Read the byte that just came in (use a loopback between MISO and MOSI
		 * to get the same byte back)
		 */
		//rx_value = spi_read(SPI2);
		/* printf the byte just received */
		//counter++;
		/* LED on/off */
		gpio_toggle(GPIOC, GPIO1);
		//uart_printf("Test mode\r\n");
		delay_ms(1000);
	}
}