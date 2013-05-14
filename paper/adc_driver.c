#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/gpio.h>

#include "adc_driver.h"

void adc_setup(void)
{
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO6);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO5 | GPIO1);

	/* Make sure the ADC doesn't run during config. */
	adc_off(ADC1);

	/* We configure everything for one single conversion. */
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	
	//adc_enable_temperature_sensor(ADC1);
	//adc_set_injected_offset(ADC1, 0x2, 0x00);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	delay_ms(170);

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);
}

u16 adc_measure(void){
	u8 channel_array[16];
	u16 val,valLoaded;

	/* Select the channel we want to convert. 16=temperature_sensor. */
	channel_array[0] = 6;
	adc_set_regular_sequence(ADC1, 1, channel_array);

	/*
	  * Start the conversion directly (not trigger mode).
	  */
	adc_start_conversion_direct(ADC1);

	/* Wait for end of conversion. */
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));

	val = ADC_DR(ADC1);
	
	return val;
	
	gpio_set(GPIOA, GPIO1);
	adc_start_conversion_direct(ADC1);

	/* Wait for end of conversion. */
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));

	valLoaded = ADC_DR(ADC1);
	
	gpio_clear(GPIOA, GPIO5 | GPIO1);
	return val - valLoaded;
}

batt_state adc_testBatt(void){
	u8 channel_array[16];
	u16 val,valLoaded;
	gpio_set(GPIOA, GPIO5);
	delay_ms(25);
	/* Select the channel we want to convert. 16=temperature_sensor. */
	channel_array[0] = 6;
	adc_set_regular_sequence(ADC1, 1, channel_array);

	/*
	  * Start the conversion directly (not trigger mode).
	  */
	adc_start_conversion_direct(ADC1);

	/* Wait for end of conversion. */
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));

	val = ADC_DR(ADC1);
	
	if(val < 0x0A3D){
	  return BATTCRIT;
	}
	val = val & 0xFFFC;
	gpio_set(GPIOA, GPIO1);
	delay_ms(25);
	adc_start_conversion_direct(ADC1);

	/* Wait for end of conversion. */
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));

	valLoaded = ADC_DR(ADC1);
	
	gpio_clear(GPIOA, GPIO5 | GPIO1);
	
	valLoaded = valLoaded & 0xFFFC;
	// kontroluje pokles vetsi nez 0,75 V po zatizeni 15 Ohm
	if(val - valLoaded > 0x1E0){
	    return BATTLOW;
	}
	
	return BATTOK;
}