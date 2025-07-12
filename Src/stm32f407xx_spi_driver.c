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

void SPI_PeripheralControl(SPI_Handle_t *pSPIHandle, uint8_t EnOrDis)
{
    if (EnOrDis)
    {
        pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
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

/* cortex m4's nvic has 8 NVIC_ISER and 8 NVIC_ICER registers, 32 bits each. so theoretically 8*32 = 256 interrupts (IRQNumbers) can be configured
but we'll consider 3 SPI_ISER and 3 SPI_ICER */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis)
{

    if (EnOrDis)
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 32 && IRQNumber <= 63)
        {
            *NVIC_ISER1 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 64 && IRQNumber <= 95)
        {
            *NVIC_ISER3 |= (1 << IRQNumber);
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 32 && IRQNumber <= 63)
        {
            *NVIC_ICER1 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 64 && IRQNumber <= 95)
        {
            *NVIC_ICER3 |= (1 << IRQNumber);
        }
    }
}

void SPI_IRQPriority(uint8_t IRQNumber, uint32_t priority)
{
    uint8_t priority_reg_number = IRQNumber / 4;
    uint8_t priority_position = IRQNumber % 4;

    *(NVIC_PR_BASEADDR + priority_reg_number) |= (priority << ((8 * priority_position) + (8 - NO_PR_BITS_IMPLEMENTED)));
}

/*
SPI_SendDataAsInterrupt logic:
step 1: save pTxbuffer, length and spi state ina global variable
step 2: set spi peripheral in 'busy in transmission' mode so that no other peripheral access this spi during transmission
step 3: enable the TXEIE bit, so that interrupt is generated by the nvic each time the tx buffer is empty
step 4: handle the interrupt in the isr
*/

/* remember: this function does'nt transmit any data i.e it does not write to the the spi data register, it only prepares the spi periph
to generate an interrupt when tx buffer is empty (initiated) by the nvic. the actual transfer is handled in the isr */

uint8_t SPI_SendDataAsInterrupt(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length) // non-blocking transmission api's
{
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BUSY_IN_TX)
    {
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLength = length;
        pSPIHandle->TxState = SPI_BUSY_IN_TX; // set spi state as busy in transmission so that no other periph acces this spi

        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE); // enable interrupt generation for tx empty event
    }

    return state;
}

uint8_t SPI_ReceiveDataAsInterrupt(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length)
{
    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BUSY_IN_RX)
    {
        state = SPI_BUSY_IN_RX;
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLength = length;

        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }

    return state;
}

/*
 IRQ handling logic:
step 1: first check what event/error caused the interrupt by checking the status register. there are 6 possible interrupts
step 2: handle each error with respective handler functions
 */

/* checking error/event type logic:
- fist check SR for error bit set/reset
- theen check in CR2 if that specific interrupt is en/dis
 */

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp1, temp2;

    /* checking for txe */

    temp1 = ((pSPIHandle->pSPIx->SR >> SPI_SR_TXE) & 0x1);     // checking sr txe bit
    temp2 = ((pSPIHandle->pSPIx->CR2 >> SPI_CR2_TXEIE) & 0x1); // checking if that interrupt is enabled (as spi interrupts are disabled by default)

    if (temp1 && temp2)
    {
        spi_txe_handler(pSPIHandle);
    }

    /* checking for rxne */

    temp1 = ((pSPIHandle->pSPIx->SR >> SPI_SR_RXNE) & 0x1);
    temp2 = ((pSPIHandle->pSPIx->CR2 >> SPI_CR2_RXNEIE) & 0x1);

    if (temp1 && temp2)
    {
        spi_rxe_handler(pSPIHandle);
    }

    /* checking overrun error */

    temp1 = ((pSPIHandle->pSPIx->SR >> SPI_SR_OVR) & 0x1);
    temp2 = ((pSPIHandle->pSPIx->CR2 >> SPI_CR2_ERRIE) & 0x1);

    if (temp1 && temp2)
    {
        spi_ovr_handler(pSPIHandle);
    }
}

static void spi_txe_handler(SPI_Handle_t *pSPIHandle)
{
    if ((pSPIHandle->pSPIx->CR1 >> SPI_CR1_DFF) & 0x1) // 16 bit dff
    {
        pSPIHandle->pSPIx->DR = *(uint16_t *)pSPIHandle->pTxBuffer;
        (uint16_t *)pSPIHandle->pTxBuffer++;
        pSPIHandle->TxLength--; // this is the reason we needed length ptxbuffer in the global scope. to access them in the handler functions
        pSPIHandle->TxLength--;
    }

    else
    {
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->pTxBuffer--;
        pSPIHandle->TxLength--;
    }

    /*
    spi transmission complete logic:
    step1: close the spi ctransmission channel
    step2: inform the application through a callback function
    */

    /*
    to close the spi;
    step1: clear the txeie bit
    step2: free pRxBuffer ptr and assign length=0
    step3: set spi state as ready
    */

    if (!pSPIHandle->TxLength) // if transmission is complete
    {
        pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
        free(pSPIHandle->pTxBuffer);
        pSPIHandle->pTxBuffer = NULL;
        pSPIHandle->TxLength = 0;
        pSPIHandle->TxState = SPI_READY;

        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
    }
}

static void spi_rxe_handler(SPI_Handle_t *pSPIHandle)
{
    if ((pSPIHandle->pSPIx->CR1 >> SPI_CR1_DFF) & 0x1)
    {
        *(uint16_t *)pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLength--;
        pSPIHandle->RxLength--;
        (uint16_t *)pSPIHandle->pRxBuffer++;
    }
    else
    {
        *(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLength--;
        pSPIHandle->RxLength--;
        pSPIHandle->pRxBuffer++;
    }

    if (!pSPIHandle->RxLength)
    { // if reception is complete
        pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
        pSPIHandle->RxLength = 0;
        free(pSPIHandle->pRxBuffer);
        pSPIHandle->pRxBuffer = NULL;
        pSPIHandle->RxState = SPI_READY;

        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
    }
}

/*
logic:
step1; clear overrun error flag when spi is not transmitting (by reading dr and sr)
step2: inform application */

static void spi_ovr_handler(SPI_Handle_t *pSPIHandle)
{
    uint32_t temp;
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
    { // you dont want to read the dr and clear it during tx
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }

    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    // weak implementation, can be overrun by the application
}

/* functions for the application to call explicitly */

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->TxLength = 0;
    free(pSPIHandle->pTxBuffer);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->RxLength = 0;
    free(pSPIHandle->pRxBuffer);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOvrFlag(SPI_RegDef_t *pSPIx){
    uint32_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
}