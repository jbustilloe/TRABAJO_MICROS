/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f407xx.h"

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
GPIO_InitTypeDef GPIO_InitStruct;



// Definiciones de pin
#define TRIGGER_PIN1  GPIO_PIN_0
#define ECHO_PIN1     GPIO_PIN_1
#define BUTTON1_PIN   GPIO_PIN_2
#define LED1_PIN      GPIO_PIN_0

#define TRIGGER_PIN2  GPIO_PIN_7
#define ECHO_PIN2     GPIO_PIN_6
#define BUTTON2_PIN   GPIO_PIN_3
#define LED2_PIN      GPIO_PIN_1

#define SENSOR1_PORT GPIOA
#define SENSOR2_PORT GPIOC
#define BUTTON_PORT  GPIOA
#define LED_PORT     GPIOB

#define SPEED_OF_SOUND 343.0f // Velocidad del sonido para el sensor de proximidad
#define TIMER_CLOCK_FREQ 1000000.0f // Frecuencia de reloj del temporizador

// Variables globales
volatile uint32_t Sensor1Duration = 0, Sensor2Duration = 0;
volatile uint8_t button1_status = 0, button2_status = 0;

// Prototipos de función
void Trigger_Sensor1(void);
void Trigger_Sensor2(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);


//static void MX_GPIO_Init(void);
//static void MX_TIM2_Init(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);


// Inicialización del TIM2
void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
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


void Trigger_Sensor1(void) {
  // Genera un pulso de 10us en el pin TRIGGER
  HAL_GPIO_WritePin(SENSOR1_PORT, TRIGGER_PIN1, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(SENSOR1_PORT, TRIGGER_PIN1, GPIO_PIN_RESET);
}

void Trigger_Sensor2(void) {
  // Genera un pulso de 10us en el pin TRIGGER
  HAL_GPIO_WritePin(SENSOR2_PORT, TRIGGER_PIN2, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(SENSOR2_PORT, TRIGGER_PIN2, GPIO_PIN_RESET);
}

// Interrupción EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static uint32_t Sensor1StartTick = 0, Sensor2StartTick = 0;

  // Pin ECHO del Sensor 1
  if (GPIO_Pin == ECHO_PIN1) {
    if (HAL_GPIO_ReadPin(SENSOR1_PORT, ECHO_PIN1) == GPIO_PIN_SET) {
      // Comenzó el pulso ECHO, así que obtenemos el tiempo de inicio
      Sensor1StartTick = HAL_GetTick();
    } else {
      // Terminó el pulso ECHO, así que calculamos la duración
      Sensor1Duration = HAL_GetTick() - Sensor1StartTick;
    }
  }

  // Pin ECHO del Sensor 2
  else if (GPIO_Pin == ECHO_PIN2) {
    if (HAL_GPIO_ReadPin(SENSOR2_PORT, ECHO_PIN2) == GPIO_PIN_SET) {
      // Comenzó el pulso ECHO, así que obtenemos el tiempo de inicio
      Sensor2StartTick = HAL_GetTick();
    } else {
      // Terminó el pulso ECHO, así que calculamos la duración
      Sensor2Duration = HAL_GetTick() - Sensor2StartTick;
    }
  }

  // Botones
  else if (GPIO_Pin == BUTTON1_PIN) {
    button1_status = !button1_status; // Cambia el estado del botón 1
    HAL_GPIO_WritePin(LED_PORT, LED1_PIN, button1_status ? GPIO_PIN_SET : GPIO_PIN_RESET); // Actualiza el LED 1
  }
  else if (GPIO_Pin == BUTTON2_PIN) {
    button2_status = !button2_status; // Cambia el estado del botón 2
    HAL_GPIO_WritePin(LED_PORT, LED2_PIN, button2_status ? GPIO_PIN_SET : GPIO_PIN_RESET); // Actualiza el LED 2
  }
}

int main(void)
{

  HAL_Init();

  /* Configure the system clock */

  SystemClock_Config();


  /* USER CODE BEGIN SysInit */
  MX_GPIO_Init();
  MX_TIM2_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    // Activar los sensores
	    Trigger_Sensor1();
	    HAL_Delay(60);
	    Trigger_Sensor2();
	    HAL_Delay(60);

	    // Calcular las distancias
	    float Sensor1Distance = (Sensor1Duration * SPEED_OF_SOUND) / (2.0f * TIMER_CLOCK_FREQ);
	    float Sensor2Distance = (Sensor2Duration * SPEED_OF_SOUND) / (2.0f * TIMER_CLOCK_FREQ);

	    // Actualizar los LEDs en función de los sensores y los botones
	    if (!button1_status && Sensor1Distance < 0.05f) {
	      // Solo se activa el LED si el botón no está activado y el sensor detecta un objeto a menos de 5 cm
	      HAL_GPIO_WritePin(LED_PORT, LED1_PIN, GPIO_PIN_SET);
	    }

	    if (!button2_status && Sensor2Distance < 0.02f) {
	      // Solo se activa el LED si el botón no está activado y el sensor detecta un objeto a menos de 2 cm
	      HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET);
	    }


  }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configure the main internal regulator output voltage

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Initialize the RCC Oscillators
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Initialize the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}



void MX_GPIO_Init(void) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Activar el Reloj GPIO */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configurar los pines GPIO */
  // Pines de TRIGGER como salida
  GPIO_InitStruct.Pin = TRIGGER_PIN1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR1_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TRIGGER_PIN2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR2_PORT, &GPIO_InitStruct);

  // Pines de ECHO como entrada
  GPIO_InitStruct.Pin = ECHO_PIN1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENSOR1_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ECHO_PIN2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENSOR2_PORT, &GPIO_InitStruct);

  // Pines de los botones como entrada
  GPIO_InitStruct.Pin = BUTTON1_PIN | BUTTON2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

  // Pines de LED como salida
  GPIO_InitStruct.Pin = LED1_PIN | LED2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  /* Configurar interrupciones EXTI */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}


void Error_Handler(void)
{
  // Manejo de errores
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
