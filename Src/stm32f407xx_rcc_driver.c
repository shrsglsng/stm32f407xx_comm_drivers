#include "stm32f407xx.h"
#include "stm32f407xx_rcc_driver.h"

/* all possible pre-scale values */

uint16_t AHB_prescaler[8] = {2, 4, 8, 16, 32, 64, 128, 256, 512}; // HPRE register

uint8_t APB1_prescaler[4] = {2, 4, 8, 16}; // PPRE1 register

uint8_t APB2_prescaler[4] = {2, 4, 8, 16}; // PPRE2 register

uint32_t RCC_GetPclk1Value()
{

    uint32_t pclk1_final, system_clk_freq;

    uint8_t clk_source, ahb_prescaler, apb1_prescaler, temp;

    /* to get system clk frequency */

    clk_source = ((RCC->CFGR >> 2) & 0x3);

    if (clk_source == 0)
    {
        system_clk_freq = 16000000; // HSI as clock source
    }
    else if (clk_source == 1)
    {
        system_clk_freq = 8000000; // HSE as clock source
    }
    else if (clk_source == 2)
    {
        system_clk_freq = RCC_GetPllOutputClk();
    }

    /* calculating AHB prescaler */

    temp = ((RCC->CFGR >> 0x4) & 0xF);
    if (temp < 8)
    {
        ahb_prescaler = 1; // no prescaler
    }
    else
    {
        ahb_prescaler = AHB_prescaler[temp - 8];
    }

    /* calculating APB1 prescaler */

    temp = ((RCC->CFGR >> 10) & 0x7);
    if (temp < 4)
    {
        apb1_prescaler = 1; // no prescaler
    }
    else
    {
        apb1_prescaler = APB1_prescaler[temp - 4];
    }

    /* calculating pclk1 frequency value */

    pclk1_final = (system_clk_freq / ahb_prescaler) / apb1_prescaler;

    return pclk1_final;
}

uint32_t RCC_GetClk2Value()
{
    uint32_t pclk2_final, system_clk_freq;
    uint8_t temp, ahb_prescaler, apb2_prescaler;

    /* calculating system_clk_freq */

    temp = ((RCC->CFGR >> 4) & 0xF);

    if (temp == 0)
    {
        system_clk_freq = 16000000;
    }
    else if (temp == 1)
    {
        system_clk_freq = 8000000;
    }
    else if (temp == 2)
    {
        system_clk_freq = RCC_GetPllOutputClk();
    }

    /* calculating ahb prescaler */

    temp = ((RCC->CFGR >> 4) & 0xF);

    if (temp < 8)
    {
        ahb_prescaler = 1;
    }
    else
    {
        ahb_prescaler = AHB_prescaler[temp - 8];
    }

    /* calculating apb2 prescaler */

    temp = ((RCC->CFGR >> 13) & 0x7);

    if (temp < 4)
    {
        apb2_prescaler = 1;
    }
    else
    {
        apb2_prescaler = APB2_prescaler[temp - 4];
    }

    pclk2_final = (system_clk_freq / ahb_prescaler) / apb2_prescaler;

    return pclk2_final;
}

uint32_t RCC_GetPllOutputClk(){
    return 0;
}