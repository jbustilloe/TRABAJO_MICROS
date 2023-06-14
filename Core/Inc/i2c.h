#ifndef I2C_H
#define I2C_H

#include "stm32f4xx_hal.h"

/* I2C handle */
extern I2C_HandleTypeDef hi2c1;

/* Function prototypes */
void I2C_Init(void);
HAL_StatusTypeDef I2C_WriteData(uint16_t devAddress, uint8_t regAddress, uint8_t* pData, uint16_t size);
HAL_StatusTypeDef I2C_ReadData(uint16_t devAddress, uint8_t regAddress, uint8_t* pData, uint16_t size);

#endif /* I2C_H */
