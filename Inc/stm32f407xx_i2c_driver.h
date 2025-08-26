#ifndef INC_STM32F407xx_I2C_DRIVER_H
#define INC_STM32F407xx_I2c_DRIVER_H

#include "stm32f407xx.h"

typedef struct
{
    uint32_t I2C_SclkSpeed;
    uint8_t I2C_DeviceAddress; // for the slave to communicate its address its address
    uint8_t I2C_AckControl;    // by default acking is disable for the first time use
    uint16_t FMDutyCycle;
} I2C_ConfigParameters_t;

typedef struct
{
    I2C_RegDef_t *xI2C;
    I2C_ConfigParameters_t *I2C_ConfigParameters;
} I2C_Handle;

void I2C_PeriClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);
void I2C_Init(I2C_Handle *I2C_Handle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle *pI2CHandle, uint8_t *pTxbuffer, uint32_t length, uint8_t slave_address);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void I2C_InterruptPriority(uint8_t IRQNumber, uint32_t Priority);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *xI2C, uint8_t FlagName);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);

#define I2C_SCL_SPEED_SLOW 100000
#define I2C_SCL_SPEED_FAST_2K 200000
#define I2C_SCL_SPEED_FAST_4K 400000

#define I2C_ACK_EN 1
#define I2C_ACK_DIS 0

#define I2C_FM_DUTY_2 0
#define I2C_FM_DUTY_16_9 1

/* i2c flag positions */

#define I2C_FLAG_SB (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF (1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10 (1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF (1 << I2C_SR1_STOPF)
#define I2C_FLAG_RxNE (1 << I2C_SR1_RxNE)
#define I2C_FLAG_TxE (1 << I2C_SR1_TxE)
#define I2C_FLAG_BERR (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR (1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR (1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT (1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT (1 << I2C_SR1_SMBALERT)

#endif INC_STM32F407xx_I2C_DRIVER_H