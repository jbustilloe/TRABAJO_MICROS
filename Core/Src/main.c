/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f407xx.h"

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
GPIO_InitTypeDef GPIO_InitStruct;



// Definición de pines para el sensor 1
#define SENSOR1_TRIGGER_PIN GPIO_PIN_0
#define SENSOR1_TRIGGER_PORT GPIOA
#define SENSOR1_ECHO_PIN GPIO_PIN_1
#define SENSOR1_ECHO_PORT GPIOA

// Definición de pines para el sensor 2
#define SENSOR2_TRIGGER_PIN GPIO_PIN_7
#define SENSOR2_TRIGGER_PORT GPIOC
#define SENSOR2_ECHO_PIN GPIO_PIN_6
#define SENSOR2_ECHO_PORT GPIOC

// Definición de pines para los LEDs
#define LED1_PIN GPIO_PIN_0
#define LED1_PORT GPIOB
#define LED2_PIN GPIO_PIN_1
#define LED2_PORT GPIOB

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

uint32_t distance1 = 0; // Variable para almacenar la distancia medida por Sensor 1
uint32_t distance2 = 0; // Variable para almacenar la distancia medida por Sensor 2

int main(void)
{
//PROBANDO PARA GITHUB
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  SystemClock_Config();

  // Habilitar el reloj del puerto GPIO
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Habilitar el reloj de GPIOA
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Habilitar el reloj de GPIOC
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN SysInit */
  MX_GPIO_Init();
  MX_TIM2_Init();

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */

  // Función para activar el trigger y enviar la señal ultrasónica
  void activateTrigger(GPIO_TypeDef* triggerPort, uint16_t triggerPin)
  {
      // Generar pulso de 10 microsegundos en el pin del trigger
      HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_SET);
      HAL_Delay(10);
      HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_RESET);
  }

  // Función para obtener la distancia medida por el sensor
  float getDistance(GPIO_TypeDef* echoPort, uint16_t echoPin)
  {
      // Implementa el código para medir el tiempo de eco y calcular la distancia
      // usando el pin de eco correspondiente al sensor
  }

  // Función para controlar los LEDs
  void controlLEDs(GPIO_TypeDef* led1Port, uint16_t led1Pin, GPIO_TypeDef* led2Port, uint16_t led2Pin)
  {
      // Encender los LEDs
      HAL_GPIO_WritePin(led1Port, led1Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(led2Port, led2Pin, GPIO_PIN_SET);

      // Esperar 3 segundos
      HAL_Delay(3000);

      // Apagar los LEDs
      HAL_GPIO_WritePin(led1Port, led1Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led2Port, led2Pin, GPIO_PIN_RESET);
  }




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Control de los LEDs mediante los pulsadores

    // Leer el estado del pulsador 1
    GPIO_PinState btn1_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);

    // Encender o apagar el LED del Área 1 según el estado del pulsador 1
    if (btn1_state == GPIO_PIN_SET)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Encender el LED del Área 1
      HAL_Delay(2000); // Mantener el LED encendido durante 6 segundos
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Apagar el LED del Área 1
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Apagar el LED del Área 1
    }

    // Leer el estado del pulsador 2
    GPIO_PinState btn2_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);

    // Encender o apagar el LED del Área 2 según el estado del pulsador 2
    if (btn2_state == GPIO_PIN_SET)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Encender el LED del Área 2
      HAL_Delay(2000); // Mantener el LED encendido durante 6 segundos
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Apagar el LED del Área 2
    }
    else
    {
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Apagar el LED del Área 2
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
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  //SENSORES PROXIMIDAD

  // Configuración del sensor 1
  GPIO_InitStruct.Pin = SENSOR1_TRIGGER_PIN; // Pin de trigger del Sensor 1
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Salida push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Sin resistencia pull
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR1_TRIGGER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SENSOR1_ECHO_PIN; // Pin de echo del Sensor 1
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Entrada
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Sin resistencia pull
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR1_ECHO_PORT, &GPIO_InitStruct);

  // Configuración del sensor 2
  GPIO_InitStruct.Pin = SENSOR2_TRIGGER_PIN; // Pin de trigger del Sensor 2
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Salida push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Sin resistencia pull
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR2_TRIGGER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SENSOR2_ECHO_PIN; // Pin de echo del Sensor 2
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Entrada
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Sin resistencia pull
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR2_ECHO_PORT, &GPIO_InitStruct);


/*  GPIO_InitStruct.Pin = GPIO_PIN_0; // Pin de trigger del Sensor 1
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Salida push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Sin resistencia pull
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1; // Pin de echo del Sensor 1
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; // Interrupción en flancos de subida y bajada
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; // Resistencia pull-down
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7; // Pin de trigger del Sensor 2
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Salida push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Sin resistencia pull
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6; // Pin de echo del Sensor 2
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; // Interrupción en flancos de subida y bajada
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; // Resistencia pull-down
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);*/


  // LEDS

  GPIO_InitStruct.Pin = GPIO_PIN_0; // Pin del LED del Área 1
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Salida push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Sin resistencia pull
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1; // Pin del LED del Área 2
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Salida push-pull
  //GPIO_InitStruct.Pull = GPIO_NOPULL; // Sin resistencia pull
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  //PULSADORES

  GPIO_InitStruct.Pin = GPIO_PIN_2; // Pin del pulsador 1
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Entrada
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Resistencia pull-up
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3; // Pin del pulsador 2
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Entrada
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Resistencia pull-up
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83; // Frecuencia de conteo de 1 MHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Configuración para el canal 1 (Sensor 1)
  sConfigIC.ICPolarity = TIM_ICPOLARITY_BOTHEDGE; // Flancos de subida y bajada
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI; // Selección directa del canal
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1; // Sin prescaler
  sConfigIC.ICFilter = 0; // Sin filtro
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  // Configuración para el canal 2 (Sensor 2)
  sConfigIC.ICPolarity = TIM_ICPOLARITY_BOTHEDGE; // Flancos de subida y bajada
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI; // Selección directa del canal
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1; // Sin prescaler
  sConfigIC.ICFilter = 0; // Sin filtro
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      distance1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Lee el valor capturado del Sensor 1
      distance1 = distance1 / 58; // Convierte el valor a distancia en cm

      // Encender el LED del Área 1 durante 6 segundos si se detecta una señal
      if (distance1 <= 5)
      {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Encender el LED del Área 1
        HAL_Delay(6000); // Mantener el LED encendido durante 6 segundos
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Apagar el LED del Área 1
      }
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      distance2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // Lee el valor capturado del Sensor 2
      distance2 = distance2 / 58; // Convierte el valor a distancia en cm

      // Encender el LED del Área 2 durante 6 segundos si se detecta una señal
      if (distance2 <= 2)
      {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Encender el LED del Área 2
        HAL_Delay(6000); // Mantener el LED encendido durante 6 segundos
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Apagar el LED del Área 2
      }
    }
  }
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
