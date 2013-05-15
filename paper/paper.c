#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/f1/bkp.h>
//project includes
#include "paper_init.h"
#include "eeprom_driver.h"
#include "spi_driver.h"
#include "uart_driver.h"
#include "adc_driver.h"
#include "paper.h"

uartBuff uart_buff;
paperSettings paper_settings;
u16 compare_time;
u16 new_time;
u32 counter = 0;
batt_state batteryWarn = BATTOK;

static void hex_to_ascii(u8 c, u8 * ret){
        u8 high, low;
	high = (c & 0xF0) >> 4;
	low = c & 0x0F;
	if(high > 9){
	  high = high - 10 + 'A';
	} else {
	  high += '0';
	}
	if(low > 9){
	  low = low - 10 + 'A';
	} else {
	  low += '0';
	}
        *ret = high;
	ret++;
	*ret = low;
}


static void debugPrint(char * debugCh){
#ifdef  VERBOSE
    uart_printf(USART1, debugCh);
#endif
}
static void debugPrintInt(int val){
#ifdef  VERBOSE
    uart_print_int(USART1, val);
#endif
}

static void init_ep_buff(void){
    uart_buff.p = 0;
    uart_buff.pointer = 0;
    uart_buff.rdy = 0;
    uart_buff.owcounter = 0;
    uart_buff.complete = 0;
    uart_buff.uartMode = IDDLE;
}

static void init_timer(void){
    timer_reset(TIM2);
    timer_set_prescaler(TIM2, 500);        // Rozlisenie 1usec
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    
    nvic_enable_irq(NVIC_TIM2_IRQ);		// Povolenie prerusenia pre TIM2
    timer_enable_irq(TIM2, TIM_DIER_CC1IE);	// Povolenie udalosti od CAPTURE
    timer_enable_irq(TIM2, TIM_DIER_UIE);	// Povolenie udalosti od PERIOD
    
    timer_set_period(TIM2, 20000);		// Inicializacia a vyvolanie
    timer_set_oc_value(TIM2, TIM_OC1, 0);	// prerusenia pre zabranenie riplov
    timer_generate_event(TIM2, TIM_SR_UIF);	// po resete
}

void tim2_isr(void)
{
    if(batteryWarn != BATTOK){
      gpio_toggle(GPIOC, GPIO1);
    }
    if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {
	timer_clear_flag(TIM2, TIM_SR_CC1IF);
    }
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
	if(batteryWarn != BATTCRIT) gpio_toggle(GPIOC, GPIO0);
#ifdef PRODUCTION
	if(counter/2 >= paper_settings.iddleTime && uart_buff.uartMode == IDDLE){
	      debugPrint("I am going to sleep now\n\n\rchrrr...");
	      //Turn off LED
	      gpio_set(GPIOC, GPIO0 | GPIO1);
	      //turn off BT
	      gpio_clear(GPIOC, GPIO4);
	      //sleep..
	      pwr_clear_wakeup_flag();
	      pwr_set_standby_mode();
	      SCB_SCR |= SCB_SCR_SLEEPDEEP;
	      __asm__("wfi");
	}
#endif
	counter++;
	timer_clear_flag(TIM2, TIM_SR_UIF);
    }
}

int main(void)
{
  	/*pwr_disable_backup_domain_write_protect();
	BKP_DR1 = 0x00;*/
    //initialize buffer
    u8 iddleTime = BKP_DR1 & 0x00FF;
    if(iddleTime > 0x05){
	paper_settings.iddleTime = iddleTime;
    } else {
	paper_settings.iddleTime = 40;
    }    
    init_ep_buff();
    clock_setup();
    gpio_setup();
    gpio_clear(GPIOC, GPIO0 | GPIO1 );
    usart_setup(&uart_buff);
    debugPrint("initializing ElectronicPaper 0.1 --- LVR --- \n\r");
    debugPrint("UART \t\t\t\t\t\t[OK]\n\r");
    spi_setup();
    debugPrint("SPI \t\t\t\t\t\t[OK]\n\r");
   /* eeprom_setup();
    debugPrint("I2C \t\t\t\t\t\t[OK]\n\r");*/
    adc_setup();
    debugPrint("ADC \t\t\t\t\t\t[OK]\n\r");
    init_timer();
    debugPrint("TIMER \t\t\t\t\t\t[OK]\n\r");

    batteryWarn = adc_testBatt();
    gpio_set(GPIOC, GPIO0 | GPIO1 );
    //turn off display, PC12 = BT reset
    gpio_clear(GPIOC, GPIO5 | GPIO6);    
    if(batteryWarn != BATTCRIT){
	//turn on BT
	gpio_set(GPIOC, GPIO4 | GPIO12);
    }
    //epClear(0xFF);
    counter = 0;
    //start timer
    debugPrint("Boot OK... I am going to main loop\n\r");
    timer_enable_counter(TIM2);      
    while (1) {
	if(uart_buff.rdy == 1){
	    debugPrint("Painting started!\n\r");
	    nvic_disable_irq(NVIC_TIM2_IRQ);
	    epSendData(&uart_buff);
	}
	if(uart_buff.uartMode != IDDLE){
	    counter = 0;
	}
	if(uart_buff.complete == 1){
	    if(uart_buff.owcounter < 2){
		//something's wrong, RED LED on
		gpio_clear(GPIOC, GPIO1);
	    }
	    //erase buffer
	    for(int c = 0;c<EP_BUFF_SIZE;c++){
	      uart_buff.buf[c] = 0x00;
	    }
	    init_ep_buff();
	    nvic_enable_irq(NVIC_TIM2_IRQ);
	    uart_buff.uartMode = IDDLE;
	}
    }
}

void delay_ms(int d){
    int i,j;
    for (j = 0; j < d; j++){
	for (i = 0; i < 4900; i++)	/* Wait a bit. */
	      __asm__("nop");
    }
}