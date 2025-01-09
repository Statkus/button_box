/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void GPIO_PinState_to_bit(uint64_t* bitfield, uint8_t bit_index, GPIO_PinState pin_state);
void Compute_Rotary_Encoder_State(Rotary_Encoder_TypeDef* rotary_encoder);
uint16_t Compute_UART_CRC(const uint8_t* buffer, uint8_t length);
uint16_t Accumulate_UART_CRC(uint16_t accumulation, uint8_t byte);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t new_ADC_data = 0;

GPIO_PinState GPIO_pin_state_debounced[NUMBER_OF_GPIOS] = {0};

uint8_t rx_buffer = 0;

uint8_t UART_buttons[5] = {0};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  Mouse_HID_TypeDef mouse_HID = {0};
  mouse_HID.report_id = 1;

  Joystick_HID_TypeDef joystick_HID = {0};
  joystick_HID.report_id = 2;

  uint8_t  previous_mouse_buttons    = 0;
  uint64_t previous_joystick_buttons = 0;

  Rotary_Encoder_TypeDef rotary_encoders[NUMBER_OF_ROTARY_ENCODERS];

  for (int i =0; i < NUMBER_OF_ROTARY_ENCODERS; i++)
  {
    rotary_encoders[i].state      = Idle;
    rotary_encoders[i].A          = GPIO_PIN_SET;
    rotary_encoders[i].B          = GPIO_PIN_SET;
    rotary_encoders[i].previous_A = GPIO_PIN_SET;
    rotary_encoders[i].previous_B = GPIO_PIN_SET;
  }

  uint32_t ADC_DMA_buffer[2] = {0};

  HAL_ADC_Start_DMA(&hadc1, ADC_DMA_buffer, 2);

  HAL_TIM_Base_Start_IT(&htim1);

  HAL_UART_Receive_IT(&huart1, &rx_buffer, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Handle mouse ------------------------------------------------------------------------------*/
    if (new_ADC_data == 1)
    {
      new_ADC_data = 0;

      if (ADC_DMA_buffer[0] > MOUSE_X_CENTER_HIGH || ADC_DMA_buffer[0] < MOUSE_X_CENTER_LOW)
      {
        mouse_HID.x = (int8_t)(MIN (ADC_DMA_buffer[0], 254) - 127) / -8;
      }
      else
      {
        mouse_HID.x = 0;
      }

      if (ADC_DMA_buffer[1] > MOUSE_Y_CENTER_HIGH || ADC_DMA_buffer[1] < MOUSE_Y_CENTER_LOW)
      {
        mouse_HID.y = (MIN (ADC_DMA_buffer[1], 254) - 127) / 8;
      }
      else
      {
        mouse_HID.y = 0;
      }

      HAL_ADC_Start_DMA(&hadc1, ADC_DMA_buffer, 2);
    }

    if (HAL_GPIO_ReadPin(MOUSE_CLICK_GPIO_Port, MOUSE_CLICK_Pin) == GPIO_PIN_RESET)
    {
      mouse_HID.buttons = 1;
    }
    else
    {
      mouse_HID.buttons = 0;
    }

    if (mouse_HID.buttons != previous_mouse_buttons || mouse_HID.x != 0 || mouse_HID.y != 0)
    {
      USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mouse_HID, sizeof(Mouse_HID_TypeDef));
      HAL_Delay(20);
    }

    previous_mouse_buttons = mouse_HID.buttons;

    /* Handle joystick ---------------------------------------------------------------------------*/
    // Compute standard buttons state
    for (int i = 0; i < NUMBER_OF_BUTTONS; i++)
    {
      GPIO_PinState_to_bit(&joystick_HID.buttons, i, GPIO_pin_state_debounced[i]);
    }

    // Compute rotary encoders state
    for (int i =0; i < NUMBER_OF_ROTARY_ENCODERS; i++)
    {
      uint8_t A_index = NUMBER_OF_BUTTONS + (i * 2);
      uint8_t B_index = NUMBER_OF_BUTTONS + (i * 2) + 1;

      rotary_encoders[i].A = GPIO_pin_state_debounced[A_index];
      rotary_encoders[i].B = GPIO_pin_state_debounced[B_index];

      Compute_Rotary_Encoder_State(&rotary_encoders[i]);

      if (rotary_encoders[i].state == Clockwise)
      {
        GPIO_PinState_to_bit(&joystick_HID.buttons, A_index, GPIO_PIN_RESET);
        GPIO_PinState_to_bit(&joystick_HID.buttons, B_index, GPIO_PIN_SET);
      }
      else if (rotary_encoders[i].state == Counterclockwise)
      {
        GPIO_PinState_to_bit(&joystick_HID.buttons, A_index, GPIO_PIN_SET);
        GPIO_PinState_to_bit(&joystick_HID.buttons, B_index, GPIO_PIN_RESET);
      }
      else
      {
        GPIO_PinState_to_bit(&joystick_HID.buttons, A_index, GPIO_PIN_SET);
        GPIO_PinState_to_bit(&joystick_HID.buttons, B_index, GPIO_PIN_SET);
      }

      rotary_encoders[i].previous_A = rotary_encoders[i].A;
      rotary_encoders[i].previous_B = rotary_encoders[i].B;
    }

    // Compute UART buttons state
    for (int i = 0; i < 5; i++)
    {
      joystick_HID.buttons &= ~(0xFF << (NUMBER_OF_GPIOS + (i * 8)));
      joystick_HID.buttons |= ((uint64_t)(UART_buttons[i]) << (NUMBER_OF_GPIOS + (i * 8)));
    }

    if (joystick_HID.buttons != previous_joystick_buttons)
    {
      USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&joystick_HID, sizeof(Joystick_HID_TypeDef));
      HAL_Delay(20);
    }

    previous_joystick_buttons = joystick_HID.buttons;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 95;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROTARY_ENCODER_1_A_Pin ROTARY_ENCODER_1_B_Pin */
  GPIO_InitStruct.Pin = ROTARY_ENCODER_1_A_Pin|ROTARY_ENCODER_1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOUSE_CLICK_Pin ROTARY_ENCODER_3_B_Pin ROTARY_ENCODER_3_A_Pin ROTARY_ENCODER_2_B_Pin
                           ROTARY_ENCODER_2_A_Pin BUTTON_17_Pin BUTTON_15_Pin BUTTON_16_Pin */
  GPIO_InitStruct.Pin = MOUSE_CLICK_Pin|ROTARY_ENCODER_3_B_Pin|ROTARY_ENCODER_3_A_Pin|ROTARY_ENCODER_2_B_Pin
                          |ROTARY_ENCODER_2_A_Pin|BUTTON_17_Pin|BUTTON_15_Pin|BUTTON_16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin BUTTON_10_Pin BUTTON_11_Pin
                           BUTTON_12_Pin BUTTON_13_Pin BUTTON_14_Pin BUTTON_3_Pin
                           BUTTON_4_Pin BUTTON_5_Pin BUTTON_6_Pin BUTTON_7_Pin
                           BUTTON_8_Pin BUTTON_9_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin|BUTTON_10_Pin|BUTTON_11_Pin
                          |BUTTON_12_Pin|BUTTON_13_Pin|BUTTON_14_Pin|BUTTON_3_Pin
                          |BUTTON_4_Pin|BUTTON_5_Pin|BUTTON_6_Pin|BUTTON_7_Pin
                          |BUTTON_8_Pin|BUTTON_9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void GPIO_PinState_to_bit(uint64_t* bitfield, uint8_t bit_index, GPIO_PinState pin_state)
{
  if (pin_state == GPIO_PIN_SET)
  {
    *bitfield &= ~(1 << bit_index);
  }
  else
  {
    *bitfield |= (1 << bit_index);
  }
}

void Compute_Rotary_Encoder_State(Rotary_Encoder_TypeDef* rotary_encoder)
{
  if (rotary_encoder->A == GPIO_PIN_RESET
      && rotary_encoder->A != rotary_encoder->previous_A
      && rotary_encoder->B == GPIO_PIN_SET)
  {
    rotary_encoder->state = Clockwise;
  }
  else if (rotary_encoder->B == GPIO_PIN_RESET
      && rotary_encoder->B != rotary_encoder->previous_B
      && rotary_encoder->A == GPIO_PIN_SET)
  {
    rotary_encoder->state = Counterclockwise;
  }
  else if ((rotary_encoder->B == GPIO_PIN_SET
            && rotary_encoder->B != rotary_encoder->previous_B
            && rotary_encoder->A == GPIO_PIN_SET)
            || (rotary_encoder->A == GPIO_PIN_SET
            && rotary_encoder->A != rotary_encoder->previous_A
            && rotary_encoder->B == GPIO_PIN_SET))
  {
    rotary_encoder->state = Idle;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    new_ADC_data = 1;
}

uint16_t Compute_UART_CRC(const uint8_t* buffer, uint8_t length)
{
  // MCRF4XX 16 bits CRC with 0xFFFF init value
  uint16_t accumulation = 0xFFFF;

  for (int i = 0; i < length; i++)
  {
    accumulation = Accumulate_UART_CRC(accumulation, buffer[i]);
  }

  return accumulation;
}

uint16_t Accumulate_UART_CRC(uint16_t accumulation, uint8_t byte)
{
  const uint16_t accumulation_LSB = accumulation & 0x00FF;
  const uint16_t accumulation_MSB = accumulation >> 8;
  uint16_t tmp = (uint16_t)(byte) ^ accumulation_LSB;

  tmp ^= (tmp << 4) & 0x00FF;
  return accumulation_MSB ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  static GPIO_TypeDef* const GPIO_port[NUMBER_OF_GPIOS] =
   {BUTTON_1_GPIO_Port,
    BUTTON_2_GPIO_Port,
    BUTTON_3_GPIO_Port,
    BUTTON_4_GPIO_Port,
    BUTTON_5_GPIO_Port,
    BUTTON_6_GPIO_Port,
    BUTTON_7_GPIO_Port,
    BUTTON_8_GPIO_Port,
    BUTTON_9_GPIO_Port,
    BUTTON_10_GPIO_Port,
    BUTTON_11_GPIO_Port,
    BUTTON_12_GPIO_Port,
    BUTTON_13_GPIO_Port,
    BUTTON_14_GPIO_Port,
    BUTTON_15_GPIO_Port,
    BUTTON_16_GPIO_Port,
    BUTTON_17_GPIO_Port,
    ROTARY_ENCODER_1_A_GPIO_Port,
    ROTARY_ENCODER_1_B_GPIO_Port,
    ROTARY_ENCODER_2_A_GPIO_Port,
    ROTARY_ENCODER_2_B_GPIO_Port,
    ROTARY_ENCODER_3_A_GPIO_Port,
    ROTARY_ENCODER_3_B_GPIO_Port};

  static const uint16_t GPIO_pin[NUMBER_OF_GPIOS] =
   {BUTTON_1_Pin,
    BUTTON_2_Pin,
    BUTTON_3_Pin,
    BUTTON_4_Pin,
    BUTTON_5_Pin,
    BUTTON_6_Pin,
    BUTTON_7_Pin,
    BUTTON_8_Pin,
    BUTTON_9_Pin,
    BUTTON_10_Pin,
    BUTTON_11_Pin,
    BUTTON_12_Pin,
    BUTTON_13_Pin,
    BUTTON_14_Pin,
    BUTTON_15_Pin,
    BUTTON_16_Pin,
    BUTTON_17_Pin,
    ROTARY_ENCODER_1_A_Pin,
    ROTARY_ENCODER_1_B_Pin,
    ROTARY_ENCODER_2_A_Pin,
    ROTARY_ENCODER_2_B_Pin,
    ROTARY_ENCODER_3_A_Pin,
    ROTARY_ENCODER_3_B_Pin};

  static GPIO_PinState previous_pin_state[NUMBER_OF_GPIOS] = {0};

  static uint8_t nb_cycles_stable[NUMBER_OF_GPIOS] = {0};

  for (int i = 0; i < NUMBER_OF_GPIOS; i++)
  {
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(GPIO_port[i], GPIO_pin[i]);

    if (previous_pin_state[i] == pin_state)
    {
      nb_cycles_stable[i]++;
    }
    else
    {
      previous_pin_state[i] = pin_state;
      nb_cycles_stable[i]   = 0;
    }

    if (nb_cycles_stable[i] >= GPIO_DEBOUNCE_CYCLES)
    {
      GPIO_pin_state_debounced[i] = pin_state;
    }
  }

  // Heartbeat LED at 1Hz
  static uint16_t LED_counter = 0;

  if (LED_counter > 500)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    LED_counter = 0;
  }
  else
  {
    LED_counter++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t buffer[9] = {0};
  static uint8_t buffer_index = 0;

  switch (buffer_index)
  {
    case 0:
      if (rx_buffer == 0xA5)
      {
        buffer[buffer_index] = rx_buffer;
        buffer_index++;
      }
      break;

    case 8:
      buffer[buffer_index] = rx_buffer;
      buffer_index = 0;

      uint16_t UART_CRC = Compute_UART_CRC(buffer, sizeof(buffer) - 2);
      if ((uint8_t)(UART_CRC & 0x00FF) == buffer[7] && (uint8_t)(UART_CRC >> 8) == buffer[8])
      {
        for (int i = 0; i < 5; i++)
        {
          UART_buttons[i] = buffer[i + 2];
        }
      }
      break;

    default:
      buffer[buffer_index] = rx_buffer;
      buffer_index++;
      break;
  }

  HAL_UART_Receive_IT(&huart1, &rx_buffer, 1);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
