
#include "adc.h"

/*	adc_init()
 * Calibrates and initializes adc to perform single conversions and generate
 * DMA requests at the end of the conversion
 * 
 * */
void adc_init(void)
{
	// Enable clocks
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;	// ADC0 clock
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;	// PTE20, PTE30 clock
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;	// PTB clock
	
	// Calibrate ADC
	adc_cal();

	// Configure ADC
	ADC0_CFG1 = 0; // Reset register
	ADC0_CFG1 |= (ADC_CFG1_MODE(1)  |  	// 12 bits mode
				  ADC_CFG1_ADICLK(0)|	// Input Bus Clock (20-25 MHz out of reset (FEI mode))
				  ADC_CFG1_ADIV(1)) ;	// Clock divide by 2 (10-12.5 MHz)
	
	ADC0_SC2 |= ADC_SC2_DMAEN_MASK;    // DMA Enable
	
	ADC0_SC3 = 0; // Reset SC3
	
	ADC0_SC1A |= ADC_SC1_ADCH(31); // Disable module
	
	//Config the ADC hardware trigger from PIT
	SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL (4) | SIM_SOPT7_ADC0PRETRGSEL_MASK;  //ADC0TRGSEL = 0100 PIT trigger 0 - bit#3-0, ADC0PRETRGSEL = 0 pre-trigger A - bit#4
}

/* adc_cal
 * Calibrates the adc
 * Returns 0 if successful calibration
 * Returns 1 otherwise
 * */
int adc_cal(void)
{
	ADC0_CFG1 |= (ADC_CFG1_MODE(0)  |  	// 8 bits mode   - ONLY IF DIFF=0
				  ADC_CFG1_ADICLK(1)|	// Input Bus Clock divided by 2 (20-25 MHz out of reset (FEI mode) / 2)
				  ADC_CFG1_ADIV(2)) ;	// Clock divide by 4 (2.5-3 MHz)
	
	ADC0_SC3 |= ADC_SC3_AVGE_MASK |		// Enable HW average
				ADC_SC3_AVGS(3)   |		// Set HW average of 32 samples
				ADC_SC3_CAL_MASK;		// Start calibration process


	
	while(ADC0_SC3 & ADC_SC3_CAL_MASK); // Wait for calibration to end
	
	if(ADC0_SC3 & ADC_SC3_CALF_MASK)	// Check for successful calibration
		return 1; 
	
	uint16_t calib = 0; // calibration variable 
	calib += ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 +
			 ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
	calib /= 2;
	calib |= 0x8000; 	// Set MSB 
	ADC0_PG = calib;
	calib = 0;
	calib += ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 +
			 ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
	calib /= 2;
	calib |= 0x8000;	// Set MSB
	ADC0_MG = calib;
	
	return 0;
}

/*unsigned short	adc_read(unsigned char ch)
 * 	Reads the specified adc channel and returns the 16 bits read value
 * 	
 * 	ch -> Number of the channel in which the reading will be performed
 * 	Returns the -> Result of the conversion performed by the adc
 * 
 * */
unsigned short adc_read(unsigned char ch)
{
	ADC0_SC1A = (ch & ADC_SC1_ADCH_MASK) | 
				(ADC0_SC1A & (ADC_SC1_AIEN_MASK) | ADC_SC1_DIFF_MASK); // Write to SC1A to start conversion
	while(ADC0_SC2 & ADC_SC2_ADACT_MASK); 	 // Conversion in progress
	while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)); // Run until the conversion is complete
	return ADC0_RA;
}
void dac_init(void){
	//When the DAC is enabled and the buffer is not enabled, 
	//the DAC module always converts the data in DAT0 to analog output voltage.
	// pin PTE30 is by default (ALT0) configured as DAC0_OUT
	//VDDA reference voltage (Use this option for the best ADC operation).
	//DACTRGSEL=0 - DAC trigger hardware selection DAC_C0_DACTRGSEL_MASK @@@@@@@@@@@@@@@@@@@@@@@
	SIM_SCGC6 |= SIM_SCGC6_DAC0_MASK; //DAC0 Clock Gate Control
	DAC0_C0 |= DAC_C0_DACEN_MASK + DAC_C0_DACRFS_MASK   +   DAC_C0_LPEN_MASK; 
}
