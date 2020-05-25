
#ifndef PIT_H_
#define PIT_H_

#include "derivative.h"
#include "adc.h"

#define ADC_CHANNEL 0 // Channel 0 (PTE20)
#define LED_RED  18 // PTB18
#define LED_GREEN  19 // PTB19

void pit_init(void);
void PIT_IRQHandler(void);
void enable_irq (int irq);


#endif /* PIT_H_ */
