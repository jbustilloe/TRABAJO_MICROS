/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_TIM2_Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN 0 */
#define TRIGGER_PIN GPIO_PIN_0
#define TRIGGER_PORT GPIOA
#define ECHO_PIN GPIO_PIN_1
#define ECHO_PORT GPIOA
#define OUTPUT_PIN GPIO_PIN_2
#define OUTPUT_PORT GPIOA
#define DISTANCE_THRESHOLD 5
#define TRIGGER_DURATION_US 10

void HC_SR04_Init(void);
void HC_SR04_Trigger(void);
float HC_SR04_GetDistance(void);
void LED_Timer_Start(void);
void LED_Timer_Stop(void);

/* Global Variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* GPIO Initialization ----------------------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_RESET);

  /* Configure GPIO pin : OUTPUT_PIN */
  GPIO_InitStruct.Pin = OUTPUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUTPUT_PORT, &GPIO_InitStruct);

  /* Initialize the HC-SR04 sensor */
  HC_SR04_Init();

  /* Initialize and start the timer */
  MX_TIM2_Init();
  HAL_TIM_Base_Start_IT(&htim2);

  /* Infinite loop */
  while (1)
  {
    float distance = HC_SR04_GetDistance();

    if (distance > 1 && distance < DISTANCE_THRESHOLD)
    {
      HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_SET);
      LED_Timer_Start();
    }
    else
    {
      HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_RESET);
      LED_Timer_Stop();
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

void LED_Timer_Start(void)
{
  HAL_TIM_Base_Start(&htim2); // Start the timer counter
}

void LED_Timer_Stop(void)
{
  HAL_TIM_Base_Stop(&htim2); // Stop the timer counter
}

void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;                    // TIM2 clock is 1 MHz (84 MHz / 84 = 1 MHz)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;  // Up counting mode
  htim2.Init.Period = 999;                      // Period of 1 ms (1 MHz / 1000 = 1 ms)
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    static uint32_t timer_count = 0;
    timer_count++;

    if (timer_count == 2000) // 2000 * 1 ms = 2000 ms = 2 seconds
    {
      HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_RESET); // Turn off the LED
      timer_count = 0;                                           // Reset the counter
    }
  }
}
