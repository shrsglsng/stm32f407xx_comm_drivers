/* file format:

- GPIO_PinConfigParameters_t; struct containg all possible configuration parameters for the peripheral

- GPIO_Handle_t; contains config struct and the registers required to implement the config

- standard macros used by GPIO_ConfigParameters_t. refer to r.m. register definations for maro value.

- API's, function declarations

*/

#ifndef INC_STM32F407xx_GPIO_DRIVER_H
#define INC_STM32F407xx_GPIO_DRIVER_H

#include "stm32f407xx.h"

/* refer to 'GPIO Features' int the reference manual to construct GPIO_PinConfigParameters_t,
but a few obvious parameters like 'input/output', 'pin number' modes will not be given under features */

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_Mode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PuPdControl;
    uint8_t GPIO_OutputType;
    uint8_t GPIO_AltFuncMode;

} GPIO_ConfigParameters_t;

/* GPIO_Handle_t logic, it contains config parameters and the register struct required to implement the parameters */

typedef struct
{
    GPIO_RegDef_t *GPIOx;
    GPIO_ConfigParameters_t GPIO_PinConfig;
} GPIO_Handle_t;

/* gpio pin numbers */

#define GPIO_PIN_0 0
#define GPIO_PIN_0 1
#define GPIO_PIN_0 2
#define GPIO_PIN_0 3
#define GPIO_PIN_0 4
#define GPIO_PIN_0 5
#define GPIO_PIN_0 6
#define GPIO_PIN_0 7
#define GPIO_PIN_0 8
#define GPIO_PIN_0 9
#define GPIO_PIN_0 10
#define GPIO_PIN_0 11
#define GPIO_PIN_0 12
#define GPIO_PIN_0 13
#define GPIO_PIN_0 14
#define GPIO_PIN_0 15

/* gpio pin modes */

#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_AF 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_RT 4 // interrupt rising-edge triggered
#define GPIO_MODE_IT_FT 5
#define GPIO_MODE_IT_RFT 5

/* gpio output speed */

#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_HIGH 2
#define GPIO_SPEED_VHIGH 3

/* gpio pull up/down config */

#define GPIO_NOPUPD 0
#define GPIO_PUPD_PU 1
#define GPIO_PUPD_PD 2

#define GPIO_OutputType_PP 0
#define GPIO_OutputType_OD 1

/* if knowing the gpio port(a,b,c..) / base address is enough, then pass pGPIOx as the only parameter, if additional
parameter config is required pass pGPIOHandle */

/* for pGPIOx only functions, port specific macros are initialised in .h */

/* init, enable, disable functions */

void GPIO_PeriClkInit(GPIO_RegDef_t *pGPIOx);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* data read / write functions */

uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ configuration and ISR handling */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif
