/*
 * DMA-PIT-ADC
 * 
 * The following example uses a PIT to start an adc conversion, 
 * once the conversion has finished it issues a DMA request and 
 * the DMA controller stores the converted value in a buffer.
 * 
 * Additional considerations:
 * Since the DMA controller is being implemented with a circular buffer
 * and this requires the address of the buffer to be 0 modular 
 * with respect the initial address, therefore a buffer with an absolute
 * address is being declared as extern in dma.h and its address is being set
 * at the linker file Project_Settings/Linker_Files/MKL25Z128_flash.ld
 * as follows 
 * 
  Define output sections 
	SECTIONS
	{
  	  Set absolute address for variable
  	  value = 0x20001000;
 */

#include "TFC.h"

uint16_t  value[ADC_READS];
volatile unsigned int state=0;
volatile unsigned int count=0;

int main(void)
{
	InitGPIO();
	RGB_LED_OFF;
	dma_init();
	adc_init();
	pit_init();
	dac_init();
	wait(); //sleep mode
	return 0;
}

void PORTD_IRQHandler(void){
	volatile unsigned int i;
	// check that the interrupt was for switch
	if (PORTD_ISFR & SW_POS) {
		if( state==0){
			//ADC mode - measures
			GREEN_LED_ON;
			PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; // Enable timer and interrupt PIT
		}
		else{
			//DAC mode - transmitting
			state=1;
		    RED_LED_ON;
			DMA_DCR1 |= DMA_DCR_EINT_MASK;	// Enable interrupt
		    PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; // Enable timer and interrupt PIT
		}
	}
	//Debounce or using PFE field
	while(!(GPIOD_PDIR & SW_POS) );// wait of release the button
	for(i=10000 ; i>0 ; i--); //delay, button debounce

	PORTD_ISFR |= 0x80;  // clear interrupt flag bit of PTD7
}
