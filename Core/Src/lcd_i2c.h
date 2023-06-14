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

#define LCD_D4_PIN GPIO_PIN_12
#define LCD_D4_PORT GPIOB
#define LCD_D5_PIN GPIO_PIN_13
#define LCD_D5_PORT GPIOB
#define LCD_D6_PIN GPIO_PIN_14
#define LCD_D6_PORT GPIOB
#define LCD_D7_PIN GPIO_PIN_15
#define LCD_D7_PORT GPIOB

#define LCD_I2C_ADDRESS 0x27

void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_SendCommand(uint8_t command);
void LCD_SendData(uint8_t data);
void LCD_ClearDisplay(void);
void LCD_ReturnHome(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *str);

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_Delay(50);
  LCD_SendCommand(0x33);
  LCD_SendCommand(0x32);
  LCD_SendCommand(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS);
  LCD_SendCommand(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
  LCD_SendCommand(LCD_CLEAR_DISPLAY);
  HAL_Delay(2);
  LCD_SendCommand(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
}

void LCD_SendCommand(uint8_t command)
{
  uint8_t high_nibble = (command & 0xF0);
  uint8_t low_nibble = (command & 0x0F) << 4;

  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (high_nibble & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (high_nibble & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (high_nibble & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (high_nibble & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (low_nibble & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (low_nibble & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (low_nibble & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (low_nibble & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_RESET);

  HAL_Delay(2);
}

void LCD_SendData(uint8_t data)
{
  uint8_t high_nibble = (data & 0xF0);
  uint8_t low_nibble = (data & 0x0F) << 4;

  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (high_nibble & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (high_nibble & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (high_nibble & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (high_nibble & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (low_nibble & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (low_nibble & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (low_nibble & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (low_nibble & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_RESET);

  HAL_Delay(2);
}

void LCD_ClearDisplay(void)
{
  LCD_SendCommand(LCD_CLEAR_DISPLAY);
  HAL_Delay(2);
}

void LCD_ReturnHome(void)
{
  LCD_SendCommand(LCD_RETURN_HOME);
  HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
  uint8_t address = 0x80;

  if (row == 1)
  {
    address += 0x40;
  }

  address += col;
  LCD_SendCommand(address);
}

void LCD_Print(const char *str)
{
  while (*str != '\0')
  {
    LCD_SendData(*str++);
  }
}

#endif
