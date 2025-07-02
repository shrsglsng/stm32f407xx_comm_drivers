/* file format:

- base addresses macros: (memory, bus-peripheral base add.)

- peripheral register mapping (xx_RegDef_t)

- peripheral definations --> peripheral base address typecasted to xxx_RegDef_t

- clock enable-disable macros for all peripherals

- gpio base address to port code

- IRQ numbers for each peripheral interrupt type

- control + status register bit position definition


*/

#ifndef STM32F407xx_H_
#define STM32F407xx_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute((weak));

/********************** Processor Specific Details **********************  */

/* ISER interrupt set-enable register*/

#define NVIC_ISER0 (__vo uint32_t *) 0xE000E100 // config irq number from 0-31
#define NVIC_ISER1 (__vo uint32_t *) 0xE000E104 // 32-63
#define NVIC_ISER2 (__vo uint32_t *) 0xE000E108 // 64-95 (there are 81 interrupts for stm32f407xx, so ISER2 is sufficient)
#define NVIC_ISER3 (__vo uint32_t *) 0xE000E10C

/* ICER interrupt clear-enable register */

#define NVIC_ICER0 (__vo uint32_t *) 0xE000E180
#define NVIC_ICER1 (__vo uint32_t *) 0xE000E184
#define NVIC_ICER2 (__vo uint32_t *) 0xE000E188
#define NVIC_ICER3 (__vo uint32_t *) 0xE000E18C

#define NVIC_PR_BASEADDR (__vo uint32_t *)0xE000E400

#define NO_PR_BITS_IMPLEMENTED 4        //only msb 4 bits are used for setting priority in the priority register, remaining are reserved. This is specific to STM

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

/********************** perpipheral register map (xx_RegDef_t) ***********************/

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

} RCC_RegDef_t;

/* System Configuration Controller register map */

typedef struct
{

    __vo uint32_t MEMRMP;
    __vo uint32_t PMC;
    __vo uint32_t EXTICR[4];
    uint32_t RESERVED1[2];
    __vo uint32_t CMPCR;
    __vo uint32_t RESERVED2[2];
    __vo uint32_t CFGR;

} SYSCFG_RegDef_t;

/* External Interrupt/Event Controller register map */

typedef struct
{
    __vo uint32_t IMR;
    __vo uint32_t EMR;
    __vo uint32_t RTSR;
    __vo uint32_t FTSR;
    __vo uint32_t SWIER;
    __vo uint32_t PR;

} EXTI_RegDef_t;

/* GPIO register map */

typedef struct
{

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
} GPIO_RegDef_t;

/* SPI register map */

typedef struct
{

    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t SR;
    __vo uint32_t DR;
    __vo uint32_t CRCPR;
    __vo uint32_t RXCRCR;
    __vo uint32_t TXCRC;
    __vo uint32_t I2SCFGR;
    __vo uint32_t I2SPR;

} SPI_RegDef_t;

/* I2S register map */

typedef struct
{
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

} I2C_RegDef_t;

/* USART register map */

typedef struct
{
    __vo uint32_t SR;
    __vo uint32_t DR;
    __vo uint32_t BRR;
    __vo uint32_t CR;
    __vo uint32_t CR2;
    __vo uint32_t CR3;
    __vo uint32_t GTPR;
} USART_RegDef_t;

/* peripheral instances are pointers, (RegDef_t *) */

/* peripheral instance to RegDef mapping */

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define GPIOJ ((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK ((GPIO_RegDef_t *)GPIOK_BASEADDR)

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASEADDR)
#define SPI5 ((SPI_RegDef_t *)SPI5_BASEADDR)
#define SPI6 ((SPI_RegDef_t *)SPI6_BASEADDR)

#define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)

#define USART1 ((USART_RegDef_t *)USART1_BASEADDR)
#define USART2 ((USART_RegDef_t *)USART2_BASEADDR)
#define USART3 ((USART_RegDef_t *)USART3_BASEADDR)
#define UART4 ((USART_RegDef_t *)UART4_BASEADDR)
#define UART5 ((USART_RegDef_t *)UART5_BASEADDR)
#define USART6 ((USART_RegDef_t *)USART6_BASEADDR)
#define UART7 ((USART_RegDef_t *)UART7_BASEADDR)
#define UART8 ((USART_RegDef_t *)UART8_BASEADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)
#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

/********************** clock enable-disable macros **********************  */

#define GPIOA_CLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN() (RCC->AHB1ENR |= (1 << 8))
/* #define GPIOJ_CLK_EN()(RCC->AHB1ENR | (1<<9))
#define GPIOK_CLK_EN()(RCC->AHB1ENR | (1<<10)) */

#define SPI1_CLK_EN() (RCC->APB2ENR |= (1 << 12));
#define SPI2_CLK_EN() (RCC->APB1ENR |= (1 << 14));
#define SPI3_CLK_EN() (RCC->APB1ENR |= (1 << 15));
#define SPI4_CLK_EN() (RCC->APB2ENR |= (1 << 13));
#define SPI5_CLK_EN() (RCC->APB2ENR |= (1 << 20));
#define SPI6_CLK_EN() (RCC->APB2ENR |= (1 << 21));

#define I2C1_CLK_EN() (RCC->APB1ENR |= (1 << 21));
#define I2C2_CLK_EN() (RCC->APB1ENR |= (1 << 22));
#define I2C3_CLK_EN() (RCC->APB1ENR |= (1 << 23));

#define USART1_CLK_EN() (RCC->APB2ENR |= (1 << 4));
#define USART2_CLK_EN() (RCC->APB1ENR |= (1 << 17));
#define USART3_CLK_EN() (RCC->APB1ENR |= (1 << 18));
#define UART4_CLK_EN() (RCC->APB1ENR |= (1 << 19));
#define UART5_CLK_EN() (RCC->APB1ENR |= (1 << 20));
#define USART6_CLK_EN() (RCC->APB2ENR |= (1 << 5));
#define UART7_CLK_EN() (RCC->APB1ENR |= (1 << 30));
#define UART8_CLK_EN() (RCC->APB1ENR |= (1 << 31));

#define SYSCFG_CLK_EN() (RCC->APB2ENR |= (1 << 14));

/* clock disable macros */

#define GPIOA_CLK_DIS() (RCC->AHB1ENR &= (0 << 0))
#define GPIOB_CLK_DIS() (RCC->AHB1ENR &= (0 << 1))
#define GPIOC_CLK_DIS() (RCC->AHB1ENR &= (0 << 2))
#define GPIOD_CLK_DIS() (RCC->AHB1ENR &= (0 << 3))
#define GPIOE_CLK_DIS() (RCC->AHB1ENR &= (0 << 4))
#define GPIOF_CLK_DIS() (RCC->AHB1ENR &= (0 << 5))
#define GPIOG_CLK_DIS() (RCC->AHB1ENR &= (0 << 6))
#define GPIOH_CLK_DIS() (RCC->AHB1ENR &= (0 << 7))
#define GPIOI_CLK_DIS() (RCC->AHB1ENR &= (0 << 8))

#define SPI1_CLK_DIS() (RCC->APB2ENR &= (0 << 12));
#define SPI2_CLK_DIS() (RCC->APB1ENR &= (0 << 14));
#define SPI3_CLK_DIS() (RCC->APB1ENR &= (0 << 15));
#define SPI4_CLK_DIS() (RCC->APB2ENR &= (0 << 13));
#define SPI5_CLK_DIS() (RCC->APB2ENR &= (0 << 20));
#define SPI6_CLK_DIS() (RCC->APB2ENR &= (0 << 21));

#define I2C1_CLK_DIS() (RCC->APB1ENR &= (0 << 21));
#define I2C2_CLK_DIS() (RCC->APB1ENR &= (0 << 22));
#define I2C3_CLK_DIS() (RCC->APB1ENR &= (0 << 23));

#define USART1_CLK_DIS() (RCC->APB2ENR &= (0 << 4));
#define USART2_CLK_DIS() (RCC->APB1ENR &= (0 << 17));
#define USART3_CLK_DIS() (RCC->APB1ENR &= (0 << 18));
#define UART4_CLK_DIS() (RCC->APB1ENR &= (0 << 19));
#define UART5_CLK_DIS() (RCC->APB1ENR &= (0 << 20));
#define USART6_CLK_DIS() (RCC->APB2ENR &= (0 << 5));
#define UART7_CLK_DIS() (RCC->APB1ENR &= (0 << 30));
#define UART8_CLK_DIS() (RCC->APB1ENR &= (0 << 31));

#define SYSCFG_CLK_DIS() (RCC->APB2ENR &= (0 << 14));

/* GPIO reset macros */
/* reason to set reset a bit is to simulate the expected reset event i.e high to low transition */

#define GPIOA_RESET()                \
    do                               \
    {                                \
        (RCC->AHB1RSTR |= (1 << 0)); \
        (RCC->AHB1RSTR & ~(1 << 0)); \
    } while (0)
#define GPIOB_RESET()                \
    do                               \
    {                                \
        (RCC->AHB1RSTR |= (1 << 1)); \
        (RCC->AHB1RSTR & ~(1 << 1)); \
    } while (0)
#define GPIOC_RESET()                \
    do                               \
    {                                \
        (RCC->AHB1RSTR |= (1 << 2)); \
        (RCC->AHB1RSTR & ~(1 << 2)); \
    } while (0)
#define GPIOD_RESET()                \
    do                               \
    {                                \
        (RCC->AHB1RSTR |= (1 << 3)); \
        (RCC->AHB1RSTR & ~(1 << 3)); \
    } while (0)
#define GPIOE_RESET()                \
    do                               \
    {                                \
        (RCC->AHB1RSTR |= (1 << 4)); \
        (RCC->AHB1RSTR & ~(1 << 4)); \
    } while (0)
#define GPIOF_RESET()                \
    do                               \
    {                                \
        (RCC->AHB1RSTR |= (1 << 5)); \
        (RCC->AHB1RSTR & ~(1 << 5)); \
    } while (0)
#define GPIOG_RESET()                \
    do                               \
    {                                \
        (RCC->AHB1RSTR |= (1 << 6)); \
        (RCC->AHB1RSTR & ~(1 << 6)); \
    } while (0)
#define GPIOH_RESET()                \
    do                               \
    {                                \
        (RCC->AHB1RSTR |= (1 << 7)); \
        (RCC->AHB1RSTR & ~(1 << 7)); \
    } while (0)
#define GPIOI_RESET()                \
    do                               \
    {                                \
        (RCC->AHB1RSTR |= (1 << 8)); \
        (RCC->AHB1RSTR & ~(1 << 8)); \
    } while (0)

/* base address to port code */

#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 : (x == GPIOB) ? 1 \
                                                 : (x == GPIOC)   ? 2 \
                                                 : (x == GPIOD)   ? 3 \
                                                 : (x == GPIOD)   ? 4 \
                                                 : (x == GPIOD)   ? 5 \
                                                 : (x == GPIOD)   ? 6 \
                                                 : (x == GPIOD)   ? 7 \
                                                 : (x == GPIOD)   ? 8 \
                                                                  : 0)

/* IRQ number of peripheral interrupts */

#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40

#define IRQ_NO_I2C1_EV 31
#define IRQ_NO_I2C1_ER 32
#define IRQ_NO_I2C2_EV 33
#define IRQ_NO_I2C2_ER 34
#define IRQ_NO_I2C3_ER 72
#define IRQ_NO_I2C3_ER 73

#define IRQ_NO_SPI1 35
#define IRQ_NO_SPI2 36
#define IRQ_NO_SPI3 51

#define IRQ_NO_USART1 37
#define IRQ_NO_USART2 38
#define IRQ_NO_USART3 39
#define IRQ_NO_UART4 52
#define IRQ_NO_UART5 53
#define IRQ_NO_USART6 71

/* control and status register bit position definition */

#define SPI_CR1_CPHA 0
#define SPI_CR1_CPOL 1
#define SPI_CR1_MSTR 2
#define SPI_CR1__BR 3
#define SPI_CR1_SPE 6
#define SPI_CR1__LSBFIRST 7
#define SPI_CR1_SSI 8
#define SPI_CR1__SSM 9
#define SPI_CR1_RXONLY 10
#define SPI_CR1_DFF 11
#define SPI_CR1_CRCNEXT 12
#define SPI_CR1_CRCEN 13
#define SPI_CR1_BIDIOE 14
#define SPI_CR1_BIDIMODE 15

#define SPI_CR2_RXDMAEN 0
#define SPI_CR2_TXDMAEN 1
#define SPI_CR2_SSOE 2
#define SPI_CR2_FRF 4
#define SPI_CR2_ERRIE 5
#define SPI_CR2_RXNEIE 6
#define SPI_CR2_TXEIE 7

#define SPI_SR_RXNE 0
#define SPI_SR_TXE 1
#define SPI_SR_CHSIDE 2
#define SPI_SR_UDR 3
#define SPI_SR_CRCERR 4
#define SPI_SR_MODF 5
#define SPI_SR_OVR 6
#define SPI_SR_BSY 7
#define SPI_SR_FRE 8

#define I2C_CR1_PE 0
#define I2C_CR1_SMBUS 1
#define I2C_CR1_SMBTYPE 3
#define I2C_CR1_ENARP 4
#define I2C_CR1_ENPEC 5
#define I2C_CR1_ENGC 6
#define I2C_CR1_NOSTRETCH 7
#define I2C_CR1_START 8
#define I2C_CR1__STOP 9
#define I2C_CR1_ACK 10
#define I2C_CR1__POS 11
#define I2C_CR1_PEC 12
#define I2C_CR1_ALERT 13
#define I2C_CR1_RESET 15

#define I2C_CR2_FREQ 0
#define I2C_CR2_ITERREN 8
#define I2C_CR2_ITEVTEN 9
#define I2C_CR2_ITBUFEN 10
#define I2C_CR2_DMAEN 11
#define I2C_CR2_LAST 12

#define I2C_OAR1_ADD0 0
#define I2C_OAR1_ADD7_1 1
#define I2C_OAR1_ADD9_8 8
#define I2C_OAR1_ADDMODE 15

#define I2C_OAR2_ENDUAL 0
#define I2C_OAR2_ADD2_7_1 1

#define I2C_SR1_SB 0
#define I2C_SR1_ADDR 1
#define I2C_SR1_BTF 2
#define I2C_SR1_ADD10 3
#define I2C_SR1_STOPF 4
#define I2C_SR1_RxNE 6
#define I2C_SR1_TxE 7
#define I2C_SR1_BERR 8
#define I2C_SR1_ARLO 9
#define I2C_SR1_AF 10
#define I2C_SR1_OVR 11
#define I2C_SR1_PECERR 12
#define I2C_SR1_TIMEOUT 14
#define I2C_SR1_SMBALERT 15

#define I2C_SR2_MSL 0
#define I2C_SR2_BUSY 1
#define I2C_SR2_TRA 2
#define I2C_SR2_GENCALL 4
#define I2C_SR2_SMBDEFAULT 5
#define I2C_SR2_SMBHOST 6
#define I2C_SR2_DUALF 7
#define I2C_SR2_PEC 8

#define I2C_CCR_CCR 0
#define I2C_CCR_DUTY 14
#define I2C_CCR_FS 15

#define USART_SR_PE 0
#define USART_SR_FE 1
#define USART_SR_NF 2
#define USART_SR_ORE 3
#define USART_SR_IDLE 4
#define USART_SR_RXNE 5
#define USART_SR_TC 6
#define USART_SR_TXE 7
#define USART_SR_LBD 8
#define USART_SR_CTS 9

#define USART_CR1_SBK 0
#define USART_CR1__RWU 1
#define USART_CR1_RE 2
#define USART_CR1_TE 3
#define USART_CR1_IDLEIE 4
#define USART_CR1_RXNEIE 5
#define USART_CR1_TCIE 6
#define USART_CR1_TXEIE 7
#define USART_CR1_PEIE 8
#define USART_CR1_PS 9
#define USART_CR1_PCE 10
#define USART_CR1_WAKE 11
#define USART_CR1_M 12
#define USART_CR1_UE 13
#define USART_CR1_OVER8 15

#define USART_CR2_ADD 0
#define USART_CR2_LBDL 5
#define USART_CR2_LBDIE 6
#define USART_CR2_LBCL 8
#define USART_CR2_CPHA 9
#define USART_CR2_CPOL 10
#define USART_CR2_CLKEN 11
#define USART_CR2_STOP 12
#define USART_CR2_LINEN 14

#define USART_CR3_EIE 0
#define USART_CR3_IREN 1
#define USART_CR3_IRLP 2
#define USART_CR3_HDSEL 3
#define USART_CR3_NACK 4
#define USART_CR3_SCEN 5
#define USART_CR3_DMAR 6
#define USART_CR3_DMAT 7
#define USART_CR3_RTSE 8
#define USART_CR3_CTSE 9
#define USART_CR3_CTSIE 10
#define USART_CR3_ONEBIT 11

/* general */

#define SET 1
#define RESET 0
#define ENABLE 1
#define DISABLE 0
#define GPIO_SET SET
#define GPIO_RESET RESET
#define FLAG_SET SET
#define FLAG_RESET RESET

/* include peripheral specific header files below */

#endif /* STM32F407xx_H_ */