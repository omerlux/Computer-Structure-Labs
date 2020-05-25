
#include "pit.h"

/* Initializes the PIT module to produce an interrupt every second
 * 
 * */
void pit_init(void)
{
	// Enable PIT clock
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Enable Green and Red LEDs clock and MUX
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB_PCR19 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Green
	
	// Turn on PIT
	PIT_MCR = 0;
	
	// Configure PIT to produce an interrupt every 1s
	PIT_LDVAL0 = 0x0C34F;	// 1/20Mhz = 50ns   (5ms/50ns)-1= 99,999 cycles or 0x01869F    ?????????????? its 10 SECONDS!@!@$!$@#$!@#$!@#$!@#$!@#$ 0x0C34F is 5 sec for 1 byte
	PIT_LDVAL1 = 0x01869F;
	//PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;  // Enable interrupt and enable timer
									//IN MAIN
	
	
	// Enable interrupt registers ISER and ICPR
	enable_irq(INT_PIT - 16);
}

/*	Handles PIT interrupt if enabled
 * 
 * 	Starts conversion in ADC0 with single ended channel 8 (PTB0) as input
 * 
 * */
void PIT_IRQHandler(void)
{	
	// Clear interrupt
	PIT_TFLG0 = PIT_TFLG_TIF_MASK;
	
	// Write to SC1A to start conversion with channel 0 PTE20
	ADC0_SC1A = (ADC_SC1_ADCH(ADC_CHANNEL) | 
				 (ADC0_SC1A & (ADC_SC1_AIEN_MASK | ADC_SC1_DIFF_MASK)));  
}

