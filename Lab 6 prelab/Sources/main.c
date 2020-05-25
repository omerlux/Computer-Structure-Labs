/*
 * main implementation: use this 'C' sample to create your own application
 *
 */


//#include "derivative.h" /* include peripheral declarations */
# include "TFC.h"
#define MUDULO_REGISTER  0x2EE0
#define ADC_MAX_CODE     4095

unsigned int LightIntensity = MUDULO_REGISTER/2;  // Global variable
volatile unsigned int k=1;
int main(void){

	ClockSetup();
	InitGPIO();
	InitPIT();
	InitADCs();
	InitUARTs();
	RGB_LED_OFF;
	while(1);
	return 0;
}

//-----------------------------------------------------------------
//  PORTD - ISR = Interrupt Service Routine
//-----------------------------------------------------------------
void PORTD_IRQHandler(void){
	volatile unsigned int i;
	// check that the interrupt was for switch
	if (PORTD_ISFR & SW_POS) {

		UARTprintf(UART0_BASE_PTR,"Menu\r\n");
		UARTprintf(UART0_BASE_PTR,"1. Blink RGB LEDs color after color with delay of 1sec \r\n");
		UARTprintf(UART0_BASE_PTR,"2. Delay increment of 50msec \r\n");
		UARTprintf(UART0_BASE_PTR,"3. Delay decrement of 50msec \r\n");
		UARTprintf(UART0_BASE_PTR,"4. Potentiometer voltage [volt] \r\n");
		UARTprintf(UART0_BASE_PTR,"5. Sleep \r\n");
		UARTprintf(UART0_BASE_PTR,"6. Quit \r\n");

	}
	//Debounce or using PFE field
	while(!(GPIOD_PDIR & SW_POS) );// wait of release the button
	for(i=10000 ; i>0 ; i--); //delay, button debounce

	PORTD_ISFR |= 0x00000080;  // clear interrupt flag bit of PTD7
}
//-----------------------------------------------------------------
//  UART0 - ISR
//-----------------------------------------------------------------
void UART0_IRQHandler(){

	uint8_t Temp;

	if(UART0_S1 & UART_S1_RDRF_MASK){ // RX buffer is full and ready for reading
		int tmp;
		Temp = UART0_D;
		switch ((Temp)) {
		case '1':
			PIT_LDVAL0 = 0x2DC6C00; // setup timer 0 for 1sec counting period
			PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;  //enable PIT0 interrupt
			break;
		case '2':
			PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;  //disable PIT0
			tmp = PIT_LDVAL0;
			if(tmp >=4292567296) // maximum ldval - 5ms
				PIT_LDVAL0 = 0xffffffff;
			else
				PIT_LDVAL0 = tmp+2400000; // +5ms (the clock is 48 Mhz
			PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;  //enable PIT0 interrupt
			break;
		case '3':
			PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;  //disable PIT0
			tmp = PIT_LDVAL0;
			if(tmp <=2400000)
				PIT_LDVAL0 = 240000;
			else
				PIT_LDVAL0 = tmp-2400000; // +5ms (the clock is 48 Mhz
			PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;  //enable PIT0 interrupt
			break;
		case '4':
			ADC0_SC1A = POT_ADC_CHANNEL | ADC_SC1_AIEN_MASK;  //POT channel is SE0 , ADC interrupt is enabled.
			break;
		case '5':
			PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;  //disable PIT0 Disabling timers
			RGB_LED_OFF;
			break;
		case '6':
			PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;  //disable PIT0 Disabling timers
			RGB_LED_OFF;
			UART0_C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK); // Disable UART0 
			break;

		}


	}
	/* NO NEED TO TRANSMIT
    if(UART0_S1 & UART_S1_TDRE_MASK){ // TX buffer is empty and ready for sending

		UART0_D = 'a';
	}
	*/
}

//-----------------------------------------------------------------
// PIT - ISR = Interrupt Service Routine
//-----------------------------------------------------------------
/*	Handles PIT interrupt if enabled
 * 
 * 	Starts RGB Led by led
 * 
 * */
void PIT_IRQHandler(void)
{	
	// Clear interrupt
	PIT_TFLG0 = PIT_TFLG_TIF_MASK;
	BLUE_LED_TOGGLE;
	switch(k){
	case 1:	BLUE_LED_ON;
			k++;
			break;
	case 2:	BLUE_LED_OFF;
			RED_LED_ON;
			k++;
			break;
	case 3:	BLUE_LED_ON;
			k++;
			break;
	case 4:	BLUE_LED_OFF;
			RED_LED_OFF;
			GREEN_LED_ON;
			k++;
			break;
	case 5:	BLUE_LED_ON;
			k++;
			break;
	case 6:	BLUE_LED_OFF;
			RED_LED_ON;
			k++;
			break;
	case 7:	BLUE_LED_ON;
			k++;
			break;
	case 8:	RGB_LED_OFF;
			k=1;
			break;
	}
}


//-----------------------------------------------------------------
// ADC0 - ISR = Interrupt Service Routine
//-----------------------------------------------------------------
void ADC0_IRQHandler(){
	
	double value = ADC0_RA;  //max is 4095
	value = ( value / 4095 )* (3.3);
	
	int A = value;
	int B = (value - A)*1000;
	char bufferA[3];
	snprintf(bufferA,3, "%d", A);
	char bufferB[6];
	if (B<10)
		snprintf(bufferB,6, "%d %d %d", 0,0, B);
	else if (B<100)
		snprintf(bufferB,6, "%d %d", 0, B);
	else
		snprintf(bufferB,6, "%d", B);
	UARTprintf(UART0_BASE_PTR,"\r\n");
	UARTprintf(UART0_BASE_PTR,"This is the voltage: ");
	UARTprintf(UART0_BASE_PTR,bufferA);
	UARTprintf(UART0_BASE_PTR,".");
	UARTprintf(UART0_BASE_PTR,bufferB);
	UARTprintf(UART0_BASE_PTR,"\r\n");
}
