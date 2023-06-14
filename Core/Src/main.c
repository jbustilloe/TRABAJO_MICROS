#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "lcd_i2c.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void GPIO_Init(void);
void TIM_Init(void);
void HC_SR04_Init(void);
void HC_SR04_Trigger(void);
float HC_SR04_GetDistance(void);
void Display_Alert(void);
void Display_SafeZone(void);

/* Global Variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
uint32_t distance_timer = 0;
uint32_t LED_timeout = 6000;  // Timeout de 3 segundos

/* LCD I2C Handle */
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN 0 */
#define TRIGGER_PIN GPIO_PIN_10
#define TRIGGER_PORT GPIOE
#define ECHO_PIN GPIO_PIN_9
#define ECHO_PORT GPIOE
#define OUTPUT_PIN GPIO_PIN_12
#define OUTPUT_PORT GPIOE
#define DISTANCE_THRESHOLD 5

/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  I2C_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* GPIO Initialization ----------------------------------------------------*/
  GPIO_Init();

  /* Initialize the HC-SR04 sensor */
  HC_SR04_Init();

  /* Initialize and start the timer */
  TIM_Init();
  HAL_TIM_Base_Start_IT(&htim2);

  /* Initialize the LCD */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000; // Configurar la velocidad de comunicación I2C a 100 kHz
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

  LCD_Init(&hi2c1);

  /* Infinite loop */
  while (1)
  {
    float distance = HC_SR04_GetDistance();

    if (distance > 0 && distance < DISTANCE_THRESHOLD)
    {
      if (HAL_GetTick() - distance_timer >= LED_timeout)
      {
        HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_RESET);
        Display_Alert();
      }
      else
      {
        HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_SET);
        Display_SafeZone();
      }
    }
    else
    {
      distance_timer = HAL_GetTick();
      HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_SET);
      Display_SafeZone();
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 144; // Configurar N para obtener una frecuencia de 72 MHz (APB1 Timer clocks)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // Configurar P para obtener una frecuencia de 36 MHz (APB1 peripheral clocks)
  RCC_OscInitStruct.PLL.PLLQ = 4; // Configurar Q para obtener una frecuencia de 18 MHz (APB2 peripheral clocks)

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  while (1)
  {
    /* Error occurred, do something */
  }
}

void GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_RESET);

  /* Configure GPIO pin : OUTPUT_PIN */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = OUTPUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUTPUT_PORT, &GPIO_InitStruct);

  /* Configure GPIO pins for I2C communication */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;   // Modo de salida en drenador abierto con resistencia de pull-up
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // Función alternativa 4 para I2C1
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;   // Modo de salida en drenador abierto con resistencia de pull-up
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // Función alternativa 4 para I2C1
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void TIM_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HC_SR04_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);

  /* Configure GPIO pin : TRIGGER_PIN */
  GPIO_InitStruct.Pin = TRIGGER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIGGER_PORT, &GPIO_InitStruct);

  /* Configure GPIO pin : ECHO_PIN */
  GPIO_InitStruct.Pin = ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);
}

void HC_SR04_Trigger(void)
{
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);
}

float HC_SR04_GetDistance(void)
{
  uint32_t start_time = 0;
  uint32_t end_time = 0;
  float distance = 0;

  /* Send trigger pulse */
  HC_SR04_Trigger();

  /* Wait for echo pulse to start */
  while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
  {
  }

  /* Measure the start time of the echo pulse */
  start_time = HAL_GetTick();

  /* Wait for echo pulse to end */
  while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
  {
  }

  /* Measure the end time of the echo pulse */
  end_time = HAL_GetTick();

  /* Calculate the duration of the echo pulse */
  uint32_t pulse_duration = end_time - start_time;

  /* Calculate the distance based on the duration of the echo pulse and the speed of sound */
  distance = (pulse_duration * 0.0343) / 2;

  return distance;
}

void Display_Alert(void)
{
	LCD_SetCursor(&hi2c1, 0, 0);
	LCD_Print(&hi2c1, "ALERTA");

}

void Display_SafeZone(void)
{
	LCD_SetCursor(&hi2c1, 0, 0);
	LCD_Print(&hi2c1, "Protegida");

}
