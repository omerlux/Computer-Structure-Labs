/*
 * main implementation: use this 'C' sample to create your own application
 *
 */


//#include "derivative.h" /* include peripheral declarations */
# include "TFC.h"
#define MUDULO_REGISTER  0x2EE0

unsigned int LightIntensity = MUDULO_REGISTER/2;  // Global variable
volatile unsigned int i=0;
volatile unsigned int curr_level=10;
volatile unsigned int Switches;
volatile unsigned int speed;
volatile unsigned int level;
volatile unsigned int move_x_levels;
volatile unsigned int direction;
volatile unsigned int j;
volatile unsigned int counter=0;
volatile unsigned int busy=0;
volatile unsigned int overflow;
volatile unsigned int time=0;
volatile unsigned int habaita;
volatile unsigned int Array [3];
volatile unsigned int* Arr;
int* EncoderSensing (int direction, int time, int level, int move_x_levels);

 int main(void){

	InitGPIO();
	ClockSetupTPM();
	MotorConfig();
	EncoderConfig();
	//InitPIT();
	InitTPM(0);
	InitTPM(1);
	InitTPM(2);
	wait(); // sleep
	return 0;
}

//-----------------------------------------------------------------
//  PORTD - ISR = Interrupt Service Routine
//-----------------------------------------------------------------
void PORTD_IRQHandler(void){ 

	// check that the interrupt was for switch
	if (PORTD_ISFR & SW_POS) {
		if(busy == 0){
			
			Switches = TFBs_GetDIP_Switch();
			if(i==0){
				i=1;
				speed=Switches;
				//Motor_Dir_Speed (0 , 0);
			}
			else if(i==1){
				i = 2;
				level = (((10)<(Switches))?(10):(Switches));
				move_x_levels = abs( curr_level - level);
				if ( curr_level > level ){ // going DOWN
					direction = 1;
				}
				else if ( curr_level < level ){
					direction = 2;
				}
				else{
					direction = 0;
				}	
				curr_level = (((10)<(Switches))?(10):(Switches));
			}
			//ACTIVATE TIMER WITH PDM AND TURN THE MOTOR ON - DIRECTION PROGRAMING IS SET
			else if(i==2){
				i=0;
				busy=1;
				overflow=0;
				
				Motor_Dir_Speed (direction , 15);	//for startup 15 speed
				TPM2_SC |= TPM_SC_CMOD(1); //Start the TPM2 counter - PWM
			
				
				//Initializing TPM0 - counter of floors
				TPM0_C2SC |= TPM_CnSC_CHIE_MASK;
				TPM0_SC |= TPM_SC_CMOD(1); //Start the TPM0 counter
				
				
				//Initializing TPM1 - time counting
				TPM1_SC |= TPM_SC_CMOD(1); //start the TPM1 counter of time
				TPM1_SC |= TPM_SC_TOIE_MASK ;   //@@@@@@@@changed from ~&
			}
		}
	}

	//Debounce or using PFE field
	while(!(GPIOD_PDIR & SW_POS) );// wait of release the button
	for(j=10000 ; j>0 ; j--); //delay, button debounce

	PORTD_ISFR |= 0x00000080;  // clear interrupt flag bit of PTD7
}


//-----------------------------------------------------------------
// TPM - ISR = Interrupt Service Routine
//-----------------------------------------------------------------
void FTM0_IRQHandler(){

	if(counter == 50)
		Motor_Dir_Speed (direction , speed);
	
	if(counter > 1177*move_x_levels)  		  				  //1,050,000 speed=0 4.5 cm    @@@@ 7cm between levels @@@@
	{
		Motor_Dir_Speed (0 , speed);
		TPM0_C2SC &= ~TPM_CnSC_CHIE_MASK;
		
		//tried to guess the number of cycles....
		TPM1_SC |= TPM_SC_CMOD(0); 							//stopping time counter
		habaita=(int)TPM1_CNT;
		time = (habaita + overflow*65535)/187;  	// 187Khz   in mili sec
		overflow = 0;
		TPM1_CNT = 1; 										//w1c
		Arr=EncoderSensing(direction,time, curr_level, move_x_levels);
		//time is DAMMN Accurate
		
		counter=0;
		busy=0;
	}

	counter = counter + 1;
	TPM0_C2SC |= TPM_CnSC_CHF_MASK; // flag is zero now

}

void FTM1_IRQHandler(){
	if(TPM1_SC & TPM_SC_TOF_MASK ){
		overflow+=1;
		TPM1_SC |= TPM_SC_TOF_MASK;     // flag is zero now
	}
	
}

//-----------------------------------------------------------------
// Encoder Sensor
//-----------------------------------------------------------------
int* EncoderSensing (int direction, int time, int level, int move_x_levels){
	
	Array[0]=direction;
	Array[1]=time/(move_x_levels*7);
	Array[2]=level;
	return *Array;
	
}
