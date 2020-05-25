/*
 * main implementation: use this 'C' sample to create your own application
 *
 */


//#include "derivative.h" /* include peripheral declarations */
# include "TFC.h"
volatile char flag = 0x01;

int main(void){
	InitGPIO();
	RGB_LED_OFF;
	//---------- VER1 - infinite Loop ------------------	
	/*	
	while(1){
	  if(flag) GREEN_LED_ON;
	  else GREEN_LED_OFF;
	}
	 */
	//---------------------------------------------------	
	stop(); // VER2 - low power mode
	return 0;
}

//-----------------------------------------------------------------
//  PORTD Input interrupt ISR
//-----------------------------------------------------------------
void PORTD_IRQHandler(void){
	volatile unsigned int i;
	int flag;
	// check that the interrupt was for switch
	if (PORTD_ISFR & SW_POS) {
		if (flag == GPIOC_PDIR){
			RED_LED_OFF;
			GREEN_LED_OFF;
			BLUE_LED_OFF;
			flag=1; // to go out of this "if"
		}
		else{
			flag=GPIOC_PDIR;
			if(GPIOC_PDIR & PORT_LOC(6) ) // 6
				RED_LED_ON; 
			else
				RED_LED_OFF;
			if(GPIOC_PDIR & PORT_LOC(5) ) //5
				GREEN_LED_ON; 
			else
				GREEN_LED_OFF;
			if(GPIOC_PDIR & PORT_LOC(4) ) //4
				BLUE_LED_ON;
			else
				BLUE_LED_OFF;
		}
	}
	//Debounce or using PFE field
	while(!(GPIOD_PDIR & SW_POS) );// wait of release the button
	for(i=10000 ; i>0 ; i--); //delay, button debounce

	PORTD_ISFR |= 0x00000080;  // clear interrupt flag bit of PTD7
}

