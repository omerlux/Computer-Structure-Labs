
#ifndef DMA_H_
#define DMA_H_

#include "derivative.h"
#include "pit.h"

#define  ADC_READS 1000      		//1000 medidot
extern uint16_t  value[ADC_READS];

void dma_init(void);
void DMA0_IRQHandler(void);

#endif /* DMA_H_ */
