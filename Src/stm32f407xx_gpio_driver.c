#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void GPIO_PeriphClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis)
{

    if (pGPIOx == GPIOA)
    {
        EnOrDis == ENABLE ? GPIOA_CLK_EN() : GPIOA_CLK_DIS();
    }
    else if (pGPIOx == GPIOB)
    {
        EnOrDis == ENABLE ? GPIOB_CLK_EN() : GPIOB_CLK_DIS();
    }
    else if (pGPIOx == GPIOC)
    {
        EnOrDis == ENABLE ? GPIOC_CLK_EN() : GPIOC_CLK_DIS();
    }
    else if (pGPIOx == GPIOD)
    {
        EnOrDis == ENABLE ? GPIOD_CLK_EN() : GPIOD_CLK_DIS();
    }
    else if (pGPIOx == GPIOE)
    {
        EnOrDis == ENABLE ? GPIOE_CLK_EN() : GPIOE_CLK_DIS();
    }
    else if (pGPIOx == GPIOF)
    {
        EnOrDis == ENABLE ? GPIOF_CLK_EN() : GPIOF_CLK_DIS();
    }
    else if (pGPIOx == GPIOG)
    {
        EnOrDis == ENABLE ? GPIOG_CLK_EN() : GPIOG_CLK_DIS();
    }
    else if (pGPIOx == GPIOH)
    {
        EnOrDis == ENABLE ? GPIOH_CLK_EN() : GPIOH_CLK_DIS();
    }
    else if (pGPIOx == GPIOI)
    {
        EnOrDis == ENABLE ? GPIOI_CLK_EN() : GPIOI_CLK_DIS();
    }
}

/* initialising logic; first clear relevant bit positions in RegDef_t , then set relevant bit position in RegDef_t */

void GPIO_Init(GPIO_Handle_t *GPIOHandle)
{

    /* initialising mode */

    /* first initialising non interrupt modes */

    if (GPIOHandle->GPIO_PinConfig.GPIO_Mode <= GPIO_MODE_ANALOG)
    {
        GPIOHandle->GPIOx->MODER &= ~(0x3 << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));      
        GPIOHandle->GPIOx->MODER |= GPIOHandle->GPIO_PinConfig.GPIO_Mode << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    /* initialising interrupt modes  */

    else{

    }

    /* configuring port speed */

    GPIOHandle->GPIOx->OSPEEDR &= ~(0x3 << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));    //clearing speed bits in gpio reg
    GPIOHandle->GPIOx->OSPEEDR |= (GPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

    /* configuring pull up pull down control */

    GPIOHandle->GPIOx->PUPDR &= ~(0x3 << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    GPIOHandle->GPIOx->PUPDR |= GPIOHandle->GPIO_PinConfig.GPIO_PuPdControl << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    /* configuring output types */

    GPIOHandle->GPIOx->OTYPER &= ~(0x3 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    GPIOHandle->GPIOx->OTYPER |= GPIOHandle->GPIO_PinConfig.GPIO_OutputType << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

    /* configuring alt func mode */

    if(GPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7){     //then config AFRL reg

        GPIOHandle->GPIOx->AFRL &= ~(0xF << (4 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        GPIOHandle->GPIOx->AFRL |= (GPIOHandle->GPIO_PinConfig.GPIO_AltFuncMode << (4 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    }

    else{       //configure AFRH reg

        GPIOHandle->GPIOx->AFRH &= ~(0xF << (4 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        GPIOHandle->GPIOx->AFRH |= (GPIOHandle->GPIO_PinConfig.GPIO_AltFuncMode << (4 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

    }


}
