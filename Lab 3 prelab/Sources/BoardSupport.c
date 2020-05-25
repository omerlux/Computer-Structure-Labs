#include "TFC.h"
#include "mcg.h"

#define MUDULO_REGISTER  0x2EE0

// set I/O for switches and LEDs
void InitGPIO()
{
	//enable Clocks to all ports - page 206, enable clock to Ports
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

	//Setup Pins as Timer Output PWM

	PORTE_PCR23 = PORT_PCR_MUX(3);  //PTE23 pin TPM2_CH1- ALT3, Edge Aligned PWM  @@@@@@ TIMER OF ENGINE



	//GPIO Configuration - DIP Switches - Input
	PORTB_PCR0 = PORT_PCR_MUX(1); // assign PTB0 as GPIO
	PORTB_PCR1 = PORT_PCR_MUX(1); // assign PTB1 as GPIO
	PORTB_PCR2 = PORT_PCR_MUX(1); // assign PTB2 as GPIO
	PORTB_PCR3 = PORT_PCR_MUX(1); // assign PTB3 as GPIO

	//GPIO Configuration - Pushbutton - Input
	PORTD_PCR7 = PORT_PCR_MUX(1); // assign PTD7 as GPIO
	GPIOD_PDDR &= ~PORT_LOC(7);  // PTD7 is Input
	
	GPIOB_PDDR &= ~PORT_LOC(0);	
	GPIOB_PDDR &= ~PORT_LOC(1);
	GPIOB_PDDR &= ~PORT_LOC(2);
	GPIOB_PDDR &= ~PORT_LOC(3);  //PTB0-3 is Input
	
	PORTB_PCR0 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK;
	PORTB_PCR1 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK;
	PORTB_PCR2 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK;
	PORTB_PCR3 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK;
	
	PORTD_PCR7 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK | PORT_PCR_IRQC(0x0a);
	enable_irq(INT_PORTD-16); // Enable Interrupts 
	set_irq_priority (INT_PORTD-16,0);  // Interrupt priority = 0 = max
}
//-----------------------------------------------------------------
// DipSwitch data reading
//-----------------------------------------------------------------
uint8_t TFBs_GetDIP_Switch()
{
	uint8_t DIP_Val=0;

	DIP_Val = (GPIOB_PDIR) & 0xF;

	return DIP_Val;
}
//-----------------------------------------------------------------
// TPMx - Initialization
//-----------------------------------------------------------------
void InitTPM(char x){  // x={0,1,2}
	switch(x){
	case 0: //NOT IN USE
		TPM0_SC = 0; // to ensure that the counter is not running
		TPM0_SC |= TPM_SC_PS(1);      //+TPM_SC_TOIE_MASK    	 //Prescaler =2, up-mode, counter-disable
		//TPM0_MOD = MUDULO_REGISTER; // PWM frequency of 1kHz = 24MHz/(2x12,000) 
		TPM0_C2SC |= TPM_CnSC_ELSB_MASK + TPM_CnSC_CHIE_MASK;
		TPM0_CONF = 0; 
		
		enable_irq(INT_TPM0-16); // Enable Interrupts 
		set_irq_priority (INT_TPM0-16,0);  // Interrupt priority = 0 = max
		break;
		
	case 1:
		TPM1_SC = 0; // to ensure that the counter is not running
		TPM1_SC |= TPM_SC_TOF_MASK + TPM_SC_PS(7); //128 divider, up count   187500 HZ
		TPM1_MOD = 0xffff;
		TPM1_CONF |= TPM_CONF_DBGMODE_MASK;
				
		enable_irq(INT_TPM1-16); // Enable Interrupts 
		set_irq_priority (INT_TPM1-16,0);  // Interrupt priority = 0 = max
		break;
		
	case 2: 
		TPM2_SC = 0; // to ensure that the counter is not running
		TPM2_SC |= TPM_SC_PS(1)+TPM_SC_TOIE_MASK; //Prescaler =2, up-mode, counter-disable
		TPM2_MOD = MUDULO_REGISTER; // PWM frequency of 1kHz = 24MHz/(2x12,000)  @@@@@@@@@@@@@@@@@@@@@ TEDER 
		TPM2_C1SC |= TPM_CnSC_MSB_MASK + TPM_CnSC_ELSB_MASK + TPM_CnSC_CHIE_MASK;
		TPM2_C1V = MUDULO_REGISTER; //@@@@@@@@@@@@@@@@@ duty cycle
		TPM2_CONF = 0;
		
		break;
	}
}
//-----------------------------------------------------------------
// TPMx - Clock Setup
//-----------------------------------------------------------------
void ClockSetupTPM(){

	pll_init(8000000, LOW_POWER, CRYSTAL,4,24,MCGOUT); //Core Clock is now at 48MHz using the 8MHZ Crystal

	//Clock Setup for the TPM requires a couple steps.
	//1st,  set the clock mux
	//See Page 124 of f the KL25 Sub-Family Reference Manual
	SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;// We Want *** MCGPLLCLK/2=24MHz *** (See Page 196 of the KL25 Sub-Family Reference Manual
	SIM_SOPT2 &= ~(SIM_SOPT2_TPMSRC_MASK); //delete the bit on TPMSRC =00  - clock disabled
	SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1); //We want the MCGPLLCLK/2 (See Page 196 of the KL25 Sub-Family Reference Manual
	//Enable the Clock to the TPM0 and PIT Modules
	//See Page 207 of f the KL25 Sub-Family Reference Manual
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK + SIM_SCGC6_TPM2_MASK + SIM_SCGC6_TPM1_MASK;;
	// TPM_clock = 24MHz , PIT_clock = 48MHz

}
//-----------------------------------------------------------------
// PIT - Initialisation
//-----------------------------------------------------------------
void InitPIT(){
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK; //Enable the Clock to the PIT Modules
	// Timer 0

	PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; //enable PIT0 and its interrupt
	PIT_MCR &= ~PIT_MCR_FRZ_MASK; // stop the pit when in debug mode
	enable_irq(INT_PIT-16); //  //Enable PIT IRQ on the NVIC
	set_irq_priority(INT_PIT-16,0);  // Interrupt priority = 0 = max
}

//-----------------------------------------------------------------
// Motor config
//-----------------------------------------------------------------
void MotorConfig(){

	//configuration of motor

	PORTC_PCR5 = PORT_PCR_MUX(1); // assign PTC5 as GPIO = M_IN_P
	PORTC_PCR6 = PORT_PCR_MUX(1); // assign PTC6 as GPIO = M_IN_N
	GPIOC_PDDR= 0x60;  // PTC5 is Output PTC6 is Output

}

//-----------------------------------------------------------------
// Motor direction and speed
//-----------------------------------------------------------------
void Motor_Dir_Speed (int Direction, int Speed){
	if(Direction == 1){ //DOWN
		GPIOC_PCOR = 0x60;  //1   LEFT
		GPIOC_PSOR = 0x40; //0
	}
	else if( Direction == 2) { //UP
		GPIOC_PCOR = 0x60;  //0   RIGHT
		GPIOC_PSOR = 0x20; //1
	}
	else{
		GPIOC_PCOR = 0x60;  //0,0   STOP
	}
	
	int DC_value = Speed * 466 + 5010;
	TPM2_C1V = DC_value;
}
//-----------------------------------------------------------------
// Encoder configuration
//-----------------------------------------------------------------
void EncoderConfig(){
	
	PORTC_PCR3 = PORT_PCR_MUX(4);  // encoder is connected to TPM0 ch2 - PORT C 3
}

/*int* EncoderSensing(int direction, int time, int level, int move_x_levels){
	
	int Array[3];
	Array[0]=direction;
	Array[1]=time/(move_x_levels*7*1000);
	Array[2]=level;
	return *Array;
	
}
*/







