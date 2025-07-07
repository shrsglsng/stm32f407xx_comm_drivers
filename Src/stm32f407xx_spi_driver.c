/* currently supports 3 spi instances */

#include "stm32f407xx.h"

void SPI_PeriClkInit(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
    if (pSPIx == SPI1)
    {
        EnOrDis == ENABLE ? SPI1_CLK_EN() : SPI1_CLK_DIS();
    }
    else if (pSPIx == SPI2)
    {
        EnOrDis == ENABLE ? SPI2_CLK_EN() : SPI2_CLK_DIS();
    }
    else if (pSPIx == SPI3)
    {
        EnOrDis == ENABLE ? SPI3_CLK_EN() : SPI3_CLK_DIS();
    }
}

void SPI_Init(SPI_Handle_t *SPI_Handle)
{

    /* configuring spi cr1 register */

    SPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR1_MSTR);
    SPI_Handle->pSPIx->CR1 |= (SPI_Handle->SPI_Config->SPI_DeviceMode << SPI_CR1_MSTR);

    if (SPI_Handle->SPI_Config->SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // bidimode re\g must be cleared
        SPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (SPI_Handle->SPI_Config->SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // bidimode must be set
        SPI_Handle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (SPI_Handle->SPI_Config->SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        // to achieve this transition from spi full duplex to rx only bt clearing bidimode + setting rxonly reg

        SPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
        SPI_Handle->pSPIx->CR1 |= (1 << SPI_CR1_RXONLY);
    }

    SPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR1_BR);
    SPI_Handle->pSPIx->CR1 |= (SPI_Handle->SPI_Config->SPI_BusConfig << SPI_CR1_BR);

    SPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR1_DFF);
    SPI_Handle->pSPIx->CR1 |= (SPI_Handle->SPI_Config->SPI_DFF << SPI_CR1_DFF);

    SPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR1_CPOL);
    SPI_Handle->pSPIx->CR1 |= (SPI_Handle->SPI_Config->SPI_CPOL << SPI_CR1_CPOL);

    SPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR1_CPHA);
    SPI_Handle->pSPIx->CR1 |= (SPI_Handle->SPI_Config->SPI_CPHA << SPI_CR1_CPHA);

    SPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR1_SSM);
    SPI_Handle->pSPIx->CR1 |= (SPI_Handle->SPI_Config->SSM << SPI_CR1_SSM);
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_RESET();
    }
    else if (pSPIx == SPI3)
    {
        SPI3_RESET();
    }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName)
{
    return ((pSPIx->SR >> FlagName) & 0x1);
}

/* SPI_SendData() logic:

step 1: check if len == 0
step 2: while len != 0 then wait unitl tx buffer is empty
step 3: check if dff is 8 or 16
step 4: if tx buffer is empty then push data into the data register + increment the buffer address
step 5: len -- or len -- -- depending of dff

*/

/*

remember firmware cannot directly acces buffers, it has to go through data registers. the flow is
firmware->data registers-> APB bus-> tx/rx buffer

*/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length) // length of msg being sent (int bytes)
{
    while (length)
    {
        while (SPI_GetStatusFlag(pSPIx, SPI_FLAG_TXE) == 0)
            ; // wait until the tx buffer is empty

        if ((pSPIx->SR >> SPI_CR1_DFF) & 0x1)
        {                                         // 16 bit format
            pSPIx->DR = *((uint16_t *)pTxBuffer); // dereferencing a typecasted pointer
            length--;
            length--;
            (uint16_t *)pTxBuffer++;
        }
        else
        {
            pSPIx->DR = *(pTxBuffer);
            length--;
            pTxBuffer++;
        }
    }
}

/* SPI_ReceiveData logic: (similar to SPI_SendData)

step 1: while len != 0
step 2: wait until rx buffer is not empty / until spi as received something
step 3: check dataframe format expected 8/16
step 4: if 16, read 2 bytes of data and increment else read 1 byte of data and increment
step 5: len--; len-- or len--

*/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t length)
{
    while (length)
    {
        while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == 0)
            ;
        if ((pSPIx->CR1 >> SPI_CR1_DFF) & 0x1)
        {
            *((uint16_t *)pRXBuffer) = pSPIx->DR;
            length--;
            length--;
            (uint16_t *)pRXBuffer++;
        }
        else
        {
            *(pRXBuffer) = pSPIx->DR;
            length--;
            pRXBuffer++;
        }
    }
}
