#include "stm32f407xx_i2c_driver.h"

void I2C_PeriClkControl(I2C_RegDef_t *xI2C, uint8_t EnOrDis)
{
    if (xI2C == I2C1)
    {
        EnOrDis ? I2C1_CLK_EN() : I2C1_CLK_DIS();
    }
    else if (xI2C == I2C2)
    {
        EnOrDis ? I2C2_CLK_EN() : I2C2_CLK_DIS();
    }
    else if (xI2C == I2C3)
    {
        EnOrDis ? I2C3_CLK_EN() : I2C3_CLK_DIS();
    }
}



void I2C_DeInit(I2C_RegDef_t *xI2C){
    if(xI2C ==I2C1){
        I2C1_RES
    }
}