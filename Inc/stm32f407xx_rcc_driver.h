#ifndef STM32F407xx_RCC_DRIVER_H
#define STM32F407xx_RCC_DRIVER_H

#include "stm32f407xx.h"

/* returns APB1 clock value */

uint32_t RCC_GetClk1Value(void);

/* returns APB2 clock value  */

uint32_t RCC_GetClk2Value(void);

uint32_t RCC_GetPllOutputClk(void);

#endif /* STM32F407xx_RCC_DRIVER_H */