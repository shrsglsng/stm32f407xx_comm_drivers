#ifndef INC_STM32F407xx_I2C_DRIVER_H
#define INC_STM32F407xx_I2c_DRIVER_H

#include "stm32f407xx.h"

typedef struct{
    uint32_t I2C_SclkSpeed;
    uint8_t I2C_DeviceAddress;  //for the slave to communicate its address its address
    uint8_t I2C_AckControl; //by default acking is disable for the first time use
    uint16_t FMDutyCycle;
} I2C_ConfigParameters_t;

typedef struct{
    I2C_RegDef_t *xI2C;
    I2C_ConfigParameters_t *I2C_ConfigParameters;
} I2C_Handle;

void I2C_PeriClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);
void I2C_Init(I2C_Handle *I2C_Handle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_InterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void I2C_InterruptPriority(uint8_t IRQNumber, uint32_t Priority);

uint8_t I2C_GetStatusFlag(I2C_RegDef_t *xI2C, uint8_t FlagName);




void I2C_PeriControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);


#define I2C_SCL_SPEED_SLOW 100000
#define I2C_SCL_SPEED_FAST_2K 200000
#define I2C_SCL_SPEED_FAST_4K 400000

#define I2C_ACK_EN 1
#define I2C_ACK_DIS 0

#define I2C_FM_DUTY_2 0
#define I2C_FM_DUTY_16_9 1


















#endif INC_STM32F407xx_I2C_DRIVER_H