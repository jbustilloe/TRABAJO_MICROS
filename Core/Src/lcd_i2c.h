#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f4xx_hal.h"

#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSOR_SHIFT 0x10
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_CGRAM_ADDR 0x40
#define LCD_SET_DDRAM_ADDR 0x80

#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

#define LCD_DISPLAY_OFF 0x00
#define LCD_DISPLAY_ON 0x04
#define LCD_CURSOR_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_BLINK_OFF 0x00
#define LCD_BLINK_ON 0x01

#define LCD_DISPLAY_SHIFT 0x08
#define LCD_CURSOR_MOVE 0x00
#define LCD_MOVE_RIGHT 0x04
#define LCD_MOVE_LEFT 0x00

#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10_DOTS 0x04
#define LCD_5x8_DOTS 0x00

#define LCD_RS_PIN GPIO_PIN_0
#define LCD_RS_PORT GPIOB
#define LCD_RW_PIN GPIO_PIN_1
#define LCD_RW_PORT GPIOB
#define LCD_EN_PIN GPIO_PIN_2
#define LCD_EN_PORT GPIOB

#define LCD_D4_PIN GPIO_PIN_4
#define LCD_D4_PORT GPIOB
#define LCD_D5_PIN GPIO_PIN_5
#define LCD_D5_PORT GPIOB
#define LCD_D6_PIN GPIO_PIN_6
#define LCD_D6_PORT GPIOB
#define LCD_D7_PIN GPIO_PIN_7
#define LCD_D7_PORT GPIOB

#define LCD_SDA_PIN GPIO_PIN_6
#define LCD_SDA_PORT GPIOB
#define LCD_SCL_PIN GPIO_PIN_7
#define LCD_SCL_PORT GPIOB


#define LCD_I2C_ADDRESS 0x3F
#define LCD_BACKLIGHT_ON 0x08
#define LCD_BACKLIGHT_OFF 0x00

void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_SendCommand(I2C_HandleTypeDef *hi2c, uint8_t command);
void LCD_SendData(I2C_HandleTypeDef *hi2c, uint8_t data);
void LCD_ClearDisplay(I2C_HandleTypeDef *hi2c);
void LCD_ReturnHome(I2C_HandleTypeDef *hi2c);
void LCD_SetCursor(I2C_HandleTypeDef *hi2c, uint8_t row, uint8_t col);
void LCD_Print(I2C_HandleTypeDef *hi2c, const char *str);

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_Delay(50);
  //uint8_t high_nibble = 0x30;
  //uint8_t low_nibble = 0x03;
  LCD_SendCommand(hi2c, 0x33);
  LCD_SendCommand(hi2c, 0x32);
  LCD_SendCommand(hi2c, LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS);
  LCD_SendCommand(hi2c, LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
  LCD_SendCommand(hi2c, LCD_CLEAR_DISPLAY);
  HAL_Delay(2);
  LCD_SendCommand(hi2c, LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
  LCD_SendCommand(hi2c, LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF | LCD_BACKLIGHT_ON);
}

void LCD_SendCommand(I2C_HandleTypeDef *hi2c, uint8_t command)
{
  uint8_t high_nibble = (command & 0xF0);
  uint8_t low_nibble = (command & 0x0F) << 4;

  uint8_t data[4];
  data[0] = high_nibble | LCD_BACKLIGHT_ON | LCD_EN_PIN;
  data[1] = high_nibble | LCD_BACKLIGHT_OFF | LCD_EN_PIN;
  data[2] = low_nibble | LCD_BACKLIGHT_ON | LCD_EN_PIN;
  data[3] = low_nibble | LCD_BACKLIGHT_OFF | LCD_EN_PIN;

  HAL_I2C_Master_Transmit(hi2c, LCD_I2C_ADDRESS, data, sizeof(data), HAL_MAX_DELAY);

  HAL_Delay(1);
}

void LCD_SendData(I2C_HandleTypeDef *hi2c, uint8_t data)
{
  uint8_t high_nibble = (data & 0xF0);
  uint8_t low_nibble = (data & 0x0F) << 4;

  uint8_t dataBuffer[4];
  dataBuffer[0] = (high_nibble | LCD_BACKLIGHT_ON | LCD_EN_PIN | LCD_RS_PIN);
  dataBuffer[1] = (high_nibble | LCD_BACKLIGHT_OFF | LCD_EN_PIN | LCD_RS_PIN);
  dataBuffer[2] = (low_nibble | LCD_BACKLIGHT_ON | LCD_EN_PIN | LCD_RS_PIN);
  dataBuffer[3] = (low_nibble | LCD_BACKLIGHT_OFF | LCD_EN_PIN | LCD_RS_PIN);

  HAL_I2C_Master_Transmit(hi2c, LCD_I2C_ADDRESS, dataBuffer, sizeof(dataBuffer), HAL_MAX_DELAY);

  HAL_Delay(1);
}

void LCD_ClearDisplay(I2C_HandleTypeDef *hi2c)
{
  LCD_SendCommand(hi2c, LCD_CLEAR_DISPLAY);
  HAL_Delay(2);
}

void LCD_ReturnHome(I2C_HandleTypeDef *hi2c)
{
  LCD_SendCommand(hi2c, LCD_RETURN_HOME);
  HAL_Delay(2);
}

void LCD_SetCursor(I2C_HandleTypeDef *hi2c, uint8_t row, uint8_t col)
{
  uint8_t address = 0x80;

  if (row == 1)
  {
    address += 0x40;
  }

  address += col;
  LCD_SendCommand(hi2c, address);
}

void LCD_Print(I2C_HandleTypeDef *hi2c, const char *str)
{
  while (*str != '\0')
  {
    LCD_SendData(hi2c, *str++);
  }
}

#endif
