#ifndef INC_STM32F407xx_SPI_DRIVER_H
#define INC_STM32F407xx_SPI_DRIVER_H

#include "stm32f407xx.h"

typedef struct
{
    uint8_t SPI_DeviceMode; // master / slave
    uint8_t SPI_BusConfig;  // full duplex, half duplex, simplex
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SSM;
} SPI_ConfigParameters_t;

typedef struct
{
    SPI_RegDef_t *pSPIx;
    SPI_ConfigParameters_t *SPI_Config;
    /* below members are needed by isr handler functions */
    uint8_t *pTxBuffer;
    uint8_t *pRxBuffer;
    uint32_t TxLength;
    uint32_t RxLength;
    uint8_t TxState;
    uint8_t RxState;

} SPI_Handle_t;

void SPI_PeriClkInit(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void SPI_IRQPriority(uint8_t IRQNumber, uint32_t Priority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName);
uint8_t SPI_SendDataAsInterrupt(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t TxLength);
uint8_t SPI_ReceiveDataAsInterrupt(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t RxLength);

/* macros for all possible values for spi config parameters */

#define SPI_MODE_SLAVE 0
#define SPI_MODE_MASTER 1

#define SPI_BUS_CONFIG_FD 0
#define SPI_BUS_CONFIG_HD 1
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 2

#define SPI_CLKSPEED_DIV2 0
#define SPI_CLKSPEED_DIV4 1
#define SPI_CLKSPEED_DIV4 2
#define SPI_CLKSPEED_DIV4 3
#define SPI_CLKSPEED_DIV4 4
#define SPI_CLKSPEED_DIV4 5
#define SPI_CLKSPEED_DIV4 6
#define SPI_CLKSPEED_DIV4 7

#define SPI_DFF_8BIT 0
#define SPI_DFF_16BIT 1

#define SPI_CPOL_LOW 0  // clk is low when idle
#define SPI_CPOL_HIGH 1 // clk is high when idle

#define SPI_CPHA_FIRST_TRANSITION 0  // data capture at first transition
#define SPI_CPHA_SECOND_TRANSITION 1 // data capture at sencond transition

#define SPI_SSM_DISABLE 0
#define SPI_SSM_ENABLE 1

/* spi flags */

#define SPI_FLAG_RXNE 0
#define SPI_FLAG_TXE 1
#define SPI_FLAG_CHSIDE 2
#define SPI_FLAG_UDR 3
#define SPI_FLAG_CRCERR 4
#define SPI_FLAG_MODF 5
#define SPI_FLAG_OVR 6
#define SPI_FLAG_BSY 7
#define SPI_FLAG_FRE 8

#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

#endif