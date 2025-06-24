/* file format:

- base addresses macros: (memory, bus-peripheral base add.)

- peripheral register mapping (xx_RegDef_t)

- peripheral definations --> peripheral base address typecasted to xxx_RegDef_t

*/

#ifndef STM32F407xx_H_
#define STM32F407xx_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute((weak));

/********************** Processor Specific Details **********************  */

/* base addresses of Flash and SRAM memories */

#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U
#define ROM_BASEADDR 0x1FFF0000U
#define SRAM SRAM1_BASEADDR

/* base addresses of Bus Peripherals (APBx, AHBx) */

#define PERIPH_BASEADDR 0x40000000U
#define APB1_BASEADDR PERIPH_BASEADDR
#define APB2_BASEADDR 0x40010000U
#define AHB1_BASEADDR 0x40020000U
#define AHB2_BASEADDR 0x50000000U
#define AHB3_BASEADDR 0x60000000U

/* peripheral base addresses associated with APB1 */

#define TIM2_BASEADDR APB1_BASEADDR
#define TIM3_BASEADDR APB1_BASEADDR + 0x0400
#define TIM4_BASEADDR APB1_BASEADDR + 0x0800
#define TIM5_BASEADDR APB1_BASEADDR + 0x0C00
#define TIM6_BASEADDR APB1_BASEADDR + 0x1000
#define TIM7_BASEADDR APB1_BASEADDR + 0x1400
#define TIM12_BASEADDR APB1_BASEADDR + 0x1800
#define TIM13_BASEADDR APB1_BASEADDR + 0x1C00
#define TIM14_BASEADDR APB1_BASEADDR + 0x2000
#define RTC_BASEADDR APB1_BASEADDR + 0x2800
#define WWDG_BASEADDR APB1_BASEADDR + 0x2C00
#define IWDG_BASEADDR APB1_BASEADDR + 0x3000
#define I2S2ext_BASEADDR APB1_BASEADDR + 0x3400
#define SPI2_I2S2_BASEADDR APB1_BASEADDR + 0x3800
#define SPI3_I2S3_BASEADDR APB1_BASEADDR + 0x3C00
#define I2S3ext_BASEADDR APB1_BASEADDR + 0x4000
#define USART2_BASEADDR APB1_BASEADDR + 0x4400
#define USART3_BASEADDR APB1_BASEADDR + 0x4800
#define UART4_BASEADDR APB1_BASEADDR + 0x4C00
#define UART5_BASEADDR APB1_BASEADDR + 0x5000
#define I2C1_BASEADDR APB1_BASEADDR + 0x5400
#define I2C2_BASEADDR APB1_BASEADDR + 0x5800
#define I2C3_BASEADDR APB1_BASEADDR + 0x5C00
#define CAN1_BASEADDR APB1_BASEADDR + 0x6400
#define CAN2_BASEADDR APB1_BASEADDR + 0x6800
#define PWR_BASEADDR APB1_BASEADDR + 0x7000
#define DAC_BASEADDR APB1_BASEADDR + 0x7400
#define UART7_BASEADDR APB1_BASEADDR + 0x7800
#define UART8_BASEADDR APB1_BASEADDR + 0x7C00

/* peripheral base addresses associated with APB2 */

#define TIM1_BASEADDR APB2_BASEADDR
#define TIM8_BASEADDR APB2_BASEADDR + 0x0400
#define USART1_BASEADDR APB2_BASEADDR + 0x1000
#define USART6_BASEADDR APB2_BASEADDR + 0x1400
#define ADC1_2_3_BASEADDR APB2_BASEADDR + 0x2000
#define SDIO_BASEADDR APB2_BASEADDR + 0x2C00
#define SPI1_BASEADDR APB2_BASEADDR + 0x3000
#define SPI4_BASEADDR APB2_BASEADDR + 0x3400
#define SYSCFG_BASEADDR APB2_BASEADDR + 0x3800
#define EXTI_BASEADDR APB2_BASEADDR + 0x3C00
#define TIM9_BASEADDR APB2_BASEADDR + 0x4000
#define TIM10_BASEADDR APB2_BASEADDR + 0x4400
#define TIM11_BASEADDR APB2_BASEADDR + 0x4800
#define SPI5_BASEADDR APB2_BASEADDR + 0x5000
#define SPI6_BASEADDR APB2_BASEADDR + 0x5400
#define SAI1_BASEADDR APB2_BASEADDR + 0x5800
#define LCD_TFT_BASEADDR APB2_BASEADDR + 0x6800

/* peripheral base address associated with AHB1 */

#define GPIOA_BASEADDR AHB1_BASEADDR
#define GPIOB_BASEADDR AHB1_BASEADDR + 0x0400
#define GPIOC_BASEADDR AHB1_BASEADDR + 0x0800
#define GPIOD_BASEADDR AHB1_BASEADDR + 0x0C00
#define GPIOE_BASEADDR AHB1_BASEADDR + 0x1000
#define GPIOF_BASEADDR AHB1_BASEADDR + 0x1400
#define GPIOG_BASEADDR AHB1_BASEADDR + 0x1800
#define GPIOH_BASEADDR AHB1_BASEADDR + 0x1C00
#define GPIOI_BASEADDR AHB1_BASEADDR + 0x2000
#define GPIOJ_BASEADDR AHB1_BASEADDR + 0x2400
#define GPIOK_BASEADDR AHB1_BASEADDR + 0x2800
#define CRC_BASEADDR AHB1_BASEADDR + 0x3000
#define RCC_BASEADDR AHB1_BASEADDR + 0x3800
#define FLASH_INTERFACE_BASEADDR AHB1_BASEADDR + 0x3C00
#define BKPSRAM_BASEADDR AHB1_BASEADDR + 0x4000
#define DMA1_BASEADDR AHB1_BASEADDR + 0x6000
#define DAM2_BASEADDR AHB1_BASEADDR + 0x6400
#define ETHERNET_BASEADDR AHB1_BASEADDR + 0x8000
#define DMA2D_BASEADDR AHB1_BASEADDR + 0xB000
#define OTG_HS_BASEADDR AHB1_BASEADDR + 0x00020000

/* peripheral base address associated with AHB2 */

#define OTG_FS_BASEADDR AHB2_BASEADDR
#define DCMI_BASEADDR AHB2_BASEADDR + 0x50000
#define CRYP_BASEADDR AHB2_BASEADDR + 0x60000
#define HASH_BASEADDR AHB2_BASEADDR + 0x60400
#define RNG_BASEADDR AHB2_BASEADDR + 0x60800

/* peripheral base addresses associated with AHB3 */

#define FSMC_FMC_BASEADDR 0xA0000000

/********************** perpipheral register map (xx_RegDef_t) **********************  */

/* Reset and Clock Control (RCC) register map */

typedef struct
{
    __vo uint32_t CR;
    __vo uint32_t PLLCFG;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSRT;
    uint32_t RESERVED0;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;
    uint32_t RESERVED2;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;
    uint32_t RESERVED4;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    uint32_t RESERVED6[2];
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
    __vo uint32_t PLLSAICFGR;
    __vo uint32_t DCKCFGR;

}RCC_RegDef_t;

/* System Configuration Controller register map */

typedef struct{

    __vo uint32_t MEMRMP;
    __vo uint32_t PMC;
    __vo uint32_t EXTICR1;
    __vo uint32_t EXTICR2;
    __vo uint32_t EXTICR3;
    __vo uint32_t EXTICR4;
    uint32_t RESERVED0[2];
    __vo uint32_t CMPCR;

}SYSCFG_RegDef_t;

/* External Interrupt/Event Controller register map */

typedef struct
{
    __vo uint32_t IMR;
    __vo uint32_t EMR;
    __vo uint32_t RTSR;
    __vo uint32_t FTSR;
    __vo uint32_t SWIER;
    __vo uint32_t PR;
    
}EXTI_RegDef_t;

typedef struct{

    __vo uint32_t MODER;
    __vo uint32_t OTYPER;
    __vo uint32_t OSPEEDR;
    __vo uint32_t PUPDR;
    __vo uint32_t IDR;
    __vo uint32_t ODR;
    __vo uint32_t BSRR;
    __vo uint32_t LCKR;
    __vo uint32_t AFRL;
    __vo uint32_t AFRH;
}GPIO_RegDef_t;

typedef struct{

    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t SR;
    __vo uint32_t DR;
    __vo uint32_t CRCPR;
    __vo uint32_t RXCRCR;
    __vo uint32_t TXCRC;
    __vo uint32_t I2SCFGR;
    __vo uint32_t I2SPR;

}SPI_RegDef_t;

typedef struct{

    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t OAR1;
    __vo uint32_t OAR2;
    __vo uint32_t DR;
    __vo uint32_t SR1;
    __vo uint32_t SR2;
    __vo uint32_t CCR;
    __vo uint32_t TRISE;
    __vo uint32_t FLTR;

}I2C_RegDef_t;





















#endif /* STM32F407xx_H_ */