#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis)
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

    else
    {
        if (GPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_IT_RT)
        {
            EXTI->RTSR |= (1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // set rising trigger reg
            EXTI->FTSR &= ~(1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // reset falling trigger reg
        }
        else if (GPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_IT_FT)
        {
            EXTI->FTSR |= (1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR &= ~(1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (GPIOHandle->GPIO_PinConfig.GPIO_Mode == GPIO_MODE_IT_RFT)
        {
            EXTI->FTSR |= (1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR |= (1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        /* now GPIOx_0 interrupt is delivered through EXTI0, GPIOx_1 through EXTI1, GPIOx_2 through EXTI2... but pin number of which port? */
        /* by default port A has control over all exti lines, this can be changed in syscfg_exticr register */

        uint8_t temp1 = GPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = GPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

        uint8_t  port_code = GPIO_BASEADDR_TO_CODE(GPIOHandle->GPIOx);

        SYSCFG_CLK_EN();

        SYSCFG->EXTICR[temp1] = port_code << (temp2 * 4);   //each exticr subset is of 4 bits

        /* enable the exti interrupt delivery using IMR */

        EXTI->IMR |= 1 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber;    //(why?)


    }

    /* configuring port speed */

    GPIOHandle->GPIOx->OSPEEDR &= ~(0x3 << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing speed bits in gpio reg
    GPIOHandle->GPIOx->OSPEEDR |= (GPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

    /* configuring pull up pull down control */

    GPIOHandle->GPIOx->PUPDR &= ~(0x3 << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    GPIOHandle->GPIOx->PUPDR |= GPIOHandle->GPIO_PinConfig.GPIO_PuPdControl << (2 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    /* configuring output types */

    GPIOHandle->GPIOx->OTYPER &= ~(0x3 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    GPIOHandle->GPIOx->OTYPER |= GPIOHandle->GPIO_PinConfig.GPIO_OutputType << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

    /* configuring alt func mode */

    if (GPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7)
    { // then config AFRL reg

        GPIOHandle->GPIOx->AFRL &= ~(0xF << (4 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        GPIOHandle->GPIOx->AFRL |= (GPIOHandle->GPIO_PinConfig.GPIO_AltFuncMode << (4 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    }

    else
    { // configure AFRH reg

        GPIOHandle->GPIOx->AFRH &= ~(0xF << (4 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        GPIOHandle->GPIOx->AFRH |= (GPIOHandle->GPIO_PinConfig.GPIO_AltFuncMode << (4 * GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

    if (pGPIOx == GPIOA)
    {
        GPIOA_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_RESET();
    }
}

uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

    uint8_t read_value;
    read_value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); // & with 00000001 so that any previous value stored towards the msb is zeroed
    return read_value;
}

uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t read_value;
    read_value = (uint16_t)pGPIOx->IDR;
    return read_value;
}

void GPIO_DigitalWriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{ // value can be HIGH LOW

    pGPIOx->ODR &= ~(0x1 << PinNumber); // reset reg first
    pGPIOx->ODR |= (value << PinNumber);
}

void GPIO_DigitalWriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
    pGPIOx->ODR &= 0x0;
    pGPIOx->ODR |= value;
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis)
{
    if (EnOrDis)
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ISER0 |= 1 << IRQNumber;
        }
        else if (IRQNumber >= 32 && IRQNumber <= 63)
        {
            *NVIC_ISER1 |= 1 << IRQNumber % 32;
        }
        else if (IRQNumber >= 64 && IRQNumber <= 95)
        {
            *NVIC_ISER2 |= 1 << IRQNumber % 64; // to index bits from 0-31 range
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ICER0 |= 1 << IRQNumber;
        }
        else if (IRQNumber >= 32 && IRQNumber <= 63)
        {
            *NVIC_ICER1 |= 1 << IRQNumber % 32;
        }
        else if (IRQNumber >= 64 && IRQNumber <= 95)
        {
            *NVIC_ICER2 |= 1 << IRQNumber % 64;
        }
    }
}

/* to remember; there are 60 priority registers, each with 4 8-bit segemets for 1 priority and in STM mcu only 4 msb bits are programmable */

void GPIO_IRQPrioityConfig(uint8_t IRQNumber, uint32_t Priority)
{
    uint8_t priority_reg_number = IRQNumber / 4;
    uint8_t priority_position = IRQNumber % 4; // there are 8 bits per position

    *(NVIC_PR_BASEADDR + priority_reg_number) |= Priority << ((8 * priority_position) + (8 - NO_PR_BITS_IMPLEMENTED));
}

/* irq handling function is specific for each peripherals, incase of gpio there are three steps for handling interrupts:

- implement isr function in main.c
- store the ISR at the vector address of the respective IRQ number (for gpio its exti 0-15). This is handled by the startup code file startup_stm32.s inside g_pfnVectors:
- clear PR (pending register)

only step 3 must be configured in the driver file

*/

void GPIO_IRQHandler(uint8_t PinNumber)
{
    if (EXTI->PR & (1 << PinNumber))
    {
        EXTI->PR |= (1 << PinNumber); // clearing the pending register is opposite of standard clear, it is write 1 to clear
    }
}