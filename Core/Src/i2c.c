#include "i2c.h"
#include "stm32f4xx_hal.h"

void Error_Handler(void);

void I2C_Init(void)
{
  /* Configure I2C peripheral settings */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

HAL_StatusTypeDef I2C_WriteData(uint16_t devAddress, uint8_t regAddress, uint8_t* pData, uint16_t size)
{
  return HAL_I2C_Mem_Write(&hi2c1, devAddress, regAddress, I2C_MEMADD_SIZE_8BIT, pData, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef I2C_ReadData(uint16_t devAddress, uint8_t regAddress, uint8_t* pData, uint16_t size)
{
  return HAL_I2C_Mem_Read(&hi2c1, devAddress, regAddress, I2C_MEMADD_SIZE_8BIT, pData, size, HAL_MAX_DELAY);
}
