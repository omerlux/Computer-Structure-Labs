#include "TFC.h"
#include "mcg.h"

#define MUDULO_REGISTER  0x2EE0

// set I/O for switches and LEDs
void InitGPIO()
{
	//enable Clocks to all ports - page 206, enable clock to Ports
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

	//GPIO Configuration - LEDs - Output
	PORTD_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;  //Blue
	GPIOD_PDDR |= BLUE_LED_LOC; //Setup as output pin	
	PORTB_PCR18 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Red  
	PORTB_PCR19 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Green
	GPIOB_PDDR |= RED_LED_LOC + GREEN_LED_LOC; //Setup as output pins
	
	//GPIO Configuration - Pushbutton - Input
	PORTD_PCR7 = PORT_PCR_MUX(1); // assign PTD7 as GPIO
	GPIOD_PDDR &= ~PORT_LOC(7);  // PTD7 is Input
	PORTD_PCR7 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK | PORT_PCR_IRQC(0x0a);
	enable_irq(INT_PORTD-16); // Enable Interrupts 
	set_irq_priority (INT_PORTD-16,0);  // Interrupt priority = 0 = max
}
//-----------------------------------------------------------------
// DipSwitch data reading
//-----------------------------------------------------------------
uint8_t TFC_GetDIP_Switch()
{
	uint8_t DIP_Val=0;
	
	DIP_Val = (GPIOC_PDIR>>4) & 0xF;

	return DIP_Val;
}
/*
//-----------------------------------------------------------------
// TPMx - Clock Setup
//-----------------------------------------------------------------
void ClockSetupTPM(){
	    
	    pll_init(8000000, LOW_POWER, CRYSTAL,4,24,MCGOUT); //Core Clock is now at 48MHz using the 8MHZ Crystal
		
	    //Clock Setup for the TPM requires a couple steps.
	    //1st,  set the clock mux
	    //See Page 124 of f the KL25 Sub-Family Reference Manual
	    SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;// We Want MCGPLLCLK/2=24MHz (See Page 196 of the KL25 Sub-Family Reference Manual
	    SIM_SOPT2 &= ~(SIM_SOPT2_TPMSRC_MASK);
	    SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1); //We want the MCGPLLCLK/2 (See Page 196 of the KL25 Sub-Family Reference Manual
		//Enable the Clock to the TPM0 and PIT Modules
		//See Page 207 of f the KL25 Sub-Family Reference Manual
		SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK + SIM_SCGC6_TPM2_MASK;
	    // TPM_clock = 24MHz , PIT_clock = 48MHz
	    
}
//-----------------------------------------------------------------
// PIT - Initialisation
//-----------------------------------------------------------------
void InitPIT(){
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK; //Enable the Clock to the PIT Modules
	// Timer 0
	PIT_LDVAL0 = 0x0000BB80; // setup timer 0 for 1msec counting period
	PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; //enable PIT0 and its interrupt
	PIT_MCR |= PIT_MCR_FRZ_MASK; // stop the pit when in debug mode
	enable_irq(INT_PIT-16); //  //Enable PIT IRQ on the NVIC
	set_irq_priority(INT_PIT-16,0);  // Interrupt priority = 0 = max
}
*/
