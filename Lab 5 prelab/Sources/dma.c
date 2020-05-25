#include "TFC.h"
#include "dma.h"

extern int count;
extern int state;
void dma_init(void)
{

	// Enable clocks
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	// Disable DMA Mux channel first
	DMAMUX0_CHCFG0 = 0x00;
	DMAMUX0_CHCFG1 = 0x00;

// Configure DMA0 ------------------------------------------------------------------------------------------
	DMA_SAR0 = (uint32_t)&ADC0_RA;
	DMA_DAR0 = (uint32_t)&value;
	// number of bytes yet to be transferred for a given block - 2 bytes(16 bits)
	DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(2000);   										//CHECK CHECK CHECK

	DMA_DCR0 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
			DMA_DCR_ERQ_MASK |		// Enable peripheral request
			DMA_DCR_CS_MASK  |		//Cycle-steal
			DMA_DCR_SSIZE(2) |		// Set source size to 16 bits
			DMA_DCR_DINC_MASK|		// Set increments to destination address
			DMA_DCR_DMOD(8)  |      // Destination address modulo of 2k Bytes    //CHECK CHECK CHECK
			DMA_DCR_DSIZE(2));		// Set destination size of 16 bits 


	//Config DMA Mux for ADC0 operation, Enable DMA channel and source
	DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(40); // Enable DMA channel and set ADC0 as source

// Configure DMA1 ------------------------------------------------------------------------------------------
	DMA_SAR1 = (uint32_t)&value;
	DMA_DAR1 = (uint32_t)&DAC0_DAT0L;
	// number of bytes yet to be transferred for a given block - 2 bytes(16 bits)
	DMA_DSR_BCR1 = DMA_DSR_BCR_BCR(2000); 

	DMA_DCR1 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
			DMA_DCR_ERQ_MASK |		// Enable peripheral request
			//DMA_DCR_CS_MASK  |	//Cycle-steal NO - continuesly
			DMA_DCR_SSIZE(2) |		// Set source size to 16 bits
			DMA_DCR_SMOD(8)  |      // Sets source modulo of 2k bytes
			DMA_DCR_SINC_MASK |     // Sets source increment of 4
			DMA_DCR_DSIZE(2)) |     // Set destination size of 8 bits 	
			DMA_DCR_AA_MASK;		// Auto Align

	DMAMUX0_CHCFG1 |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(60) | DMAMUX_CHCFG_TRIG_MASK; // periodic trigger mode, 60 is for the DEMUX

	// Enable interrupt
	enable_irq(INT_DMA0 - 16);
	enable_irq(INT_DMA1 - 16);
}

/*
 * Handles DMA0 interrupt
 * Resets the BCR register and clears the DONE flag
 * */
void DMA0_IRQHandler(void){
	PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK; 		// Disable timer PIT
	DMA_DCR0 &= ~DMA_DCR_EINT_MASK;			// Disable interrupt - NEEDED?
	DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;	// Clear Done Flag
	RGB_LED_OFF;							// RGB OFF
	ADC0_SC1A = ADC_SC1_ADCH(31);			// Disable ADC
	DMAMUX0_CHCFG0 = 0x00;					// Disable DMA Mux channel first
	//SIM_SOPT7 = 0;							// Unbound DAC and PIT
	state=1;
}

void DMA1_IRQHandler(void)
{
	// Enable DMA1 
	DMA_DSR_BCR1 |= DMA_DSR_BCR_DONE_MASK;	// Clear Done Flag
	DMA_DSR_BCR1 |= DMA_DSR_BCR_BCR(2000);		// Set byte count register
	//ready = ready+1;
}
