/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "ssd1306.h"
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define PWM_PERIOD_ARR 1679
#define DUTY_50_PERCENT 840
uint8_t  soft_start_complete = 0;    // 0 = Ramping up, 1 = Done running soft start
int32_t  soft_start_limit    = 0;    // Starts at 0% duty (0 ticks)
int32_t  SOFT_START_STEP     = 50;   // Ramps up ~1% duty per loop cycle
int32_t  MAX_ALLOWED_RAMP    = 1595; // Absolute upper bound (~95% duty)
int32_t temp = 0;
uint8_t  system_booted       = 0;
uint32_t target_voltage_adc   = 700;  // eg Setpoint for 30V
uint32_t target_current_adc   = 800;  // eg Setpoint for 10A
uint32_t current_v_reading    = 0;
uint32_t current_i_reading    = 0;

float Kp_v = 0.40f;
float Ki_v = 0.01f;
float Kd_v = 0.01f;
float error_v = 0.0f;
float integral_v = 0.0f;
float diff_v = 0.0f;
int32_t duty_request_voltage = DUTY_50_PERCENT;

float Kp_i = 0.40f;                    // Current loop is faster; start with lower gains
float Ki_i = 0.01f;
float Kd_i = 0.01f;
float error_i = 0.0f;
float integral_i = 0.0f;
float diff_i = 0.0f;
int32_t duty_request_current = DUTY_50_PERCENT;

int32_t final_ccr_target = DUTY_50_PERCENT;
volatile uint16_t adc_raw_dma[4] = {0, 0, 0, 0};

// Live Telemetry Registers
volatile uint32_t average_adc_vfb = 0;
volatile uint32_t average_adc_ifb = 0;
volatile int32_t manual_ccr = 840;

char* operating_mode = "STARTUP";

char tx_buffer[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  char *boot_msg = "\r\n=== DUAL CHANNEL SERIAL SYSTEM ONLINE ===\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)boot_msg, strlen(boot_msg), 100);
  	TIM2->CCR1 = PWM_PERIOD_ARR;

    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    __HAL_TIM_MOE_ENABLE(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_dma, 4);

      HAL_Delay(5); // Small pause to show the splash screen
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t target_v = 0;
	  uint32_t target_i = 0;
	  uint32_t v_accumulator = 0;
	  uint32_t i_accumulator = 0;
	  	  for (int i = 0; i < 50; i++)
	  	        {
	  		  	  	target_v += adc_raw_dma[0];
	  		  	  	target_i += adc_raw_dma[1];
	  	            v_accumulator += adc_raw_dma[2]; // Read voltage position
	  	            i_accumulator += adc_raw_dma[3]; // Read current position
	  	            HAL_Delay(1); // Tiny spacing between reads to gather unique conversion cycles
	  	        }
	  	  	 uint32_t target_v_raw = target_v/50;
	  	  	 uint32_t target_i_raw = target_i/50;
	  	    current_v_reading = v_accumulator / 50;
	  	    current_i_reading = i_accumulator / 50;

	  	  target_voltage_adc = 700 + ((target_v_raw * 1622) / 4095);
	  	    target_current_adc = 0 + ((target_i_raw * 1023)/4095);

	  	      error_v = (float)((int32_t)target_voltage_adc - (int32_t)current_v_reading);
	  	      	  	            integral_v += error_v;
	  	      	  	            diff_v -= error_v;
	  	      	  	            if (integral_v > 10000.0f)  integral_v = 10000.0f;
	  	      	  	            if (integral_v < -10000.0f) integral_v = -10000.0f;
	  	      	  	            if (diff_v > 10000.0f)  diff_v = 10000.0f;
	  	      	  	          	if (diff_v < -10000.0f) diff_v = -10000.0f;
	  	      	  	            duty_request_voltage += (int32_t)((Kp_v * error_v) + (Ki_v * integral_v) + (Kd_v * diff_v));
	  	      	  	            if (duty_request_voltage > 1595) duty_request_voltage = 1595;
	  	      	  	      	  	if (duty_request_voltage < 0)   duty_request_voltage = 0;
	  	       error_i = (float)((int32_t)target_current_adc - (int32_t)current_i_reading);
	  	      	  	      	  	integral_i += error_i;
	  	      	  	      	  	diff_i -= error_i;
	  	      	  	      	  	if (integral_i > 2000.0f)  integral_i = 2000.0f;
	  	      	  	      	    if (integral_i < -2000.0f) integral_i = -2000.0f;
								if (diff_i > 2000.0f)  diff_i = 2000.0f;
								if (diff_i < -2000.0f) diff_i = -2000.0f;
	  	      	  	      	  	duty_request_current += (int32_t)((Kp_i * error_i) + (Ki_i * integral_i) + (Kd_i * diff_i));
	  	      	  	      	  	if (duty_request_current > 1595) {
	  	      	  	      	  		duty_request_current = 1595;
	  	      	  	      	  		//current_i_reading =6;
	  	      	  	      	  	}
	  	      	  	      	  	if (duty_request_current < 5)   duty_request_current = 5;
	  	      	  	      //sprintf(tx_buffer, "VOLTAGE ADC (IN1): %4lu\r\nduty: %ld\r\nCurrent ADC (IN2): %4lu\r\n",
	  	      	  	      	  //	          current_v_reading, duty_request_current,current_i_reading);

	  	      	  	      	  	//        HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);

	  	      if ((current_v_reading < 20))
	  	      	  	            {
	  	      	  	                // AC is OFF: Keep the gate drive safely dead and freeze the soft-start state
	  	      	  	                //final_ccr_target = 0;
	  	      	  	                soft_start_limit = 0;
	  	      	  	                soft_start_complete = 0;
	  	      	  	                operating_mode = "OFF (NO AC)";
	  	      	  	              //  integral_v = 0.0f;
	  	      	  	                //diff_v = 0.0f;
	  	      	  	               // integral_i = 0.0f;
	  	      	  	          HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);
	  	      	  	            }
	  	      	  	      else if(soft_start_complete == 0)
	  	      	  	      	  	        	{
											    operating_mode = "SOFT START";
	  	      	  	      	  	                // Increment the artificial duty ceiling slowly on each execution frame
	  	      	  	      	  	                soft_start_limit += SOFT_START_STEP;

	  	      	  	      	  	                // The PI loop outputs are ignored during ramp-up; we anchor directly to the limit
	  	      	  	      	  	                final_ccr_target = soft_start_limit;
  	      	  	      	  	                	if (duty_request_voltage < duty_request_current)
													temp = duty_request_voltage;
												else
													temp = duty_request_current;

	  	      	  	      	  	                // Keep the integral accumulators fully cleared to prevent pre-regulation windup
	  	      	  	      	  	              //  integral_v = 0.0f;
	  	      	  	      	  	              //  integral_i = 0.0f;
	  	      	  	      	  	               // diff_v = 0.0f;
	  	      	  	      	  	                // Check if our soft-start limit has safely reached the target running ceiling
	  	      	  	      	  	                if ((soft_start_limit >= temp))
	  	      	  	      	  	                {
	  	      	  	      	  	                	if (duty_request_voltage < duty_request_current)
														soft_start_limit = duty_request_voltage;
													else
														soft_start_limit = duty_request_current;
	  	      	  	      	  	                    soft_start_complete = 1; // Soft start finished! Hand over to the system states next loop
	  	      	  	      	  	                }
	  	      	  	      	  	                HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);
	  	      	  	      	  	               // HAL_Delay(50);
	  	      	  	      	  	            }
	  	      	  	      else
	  	      	  	      {
	  	  // 1. STATE SELECTION
	  	        if (system_booted == 0 && ((current_i_reading >= target_current_adc)||(current_v_reading >= target_voltage_adc)))
	  	        {
	  	            system_booted = 1; // Transition to PI mode permanently
	  	            integral_i = 0.0f; // Clear windup for a clean start
	  	            integral_v = 0.0f;
	  	            diff_i = 0.0f;
	  	            diff_v = 0.0f;
	  	        }

	  	        if (system_booted == 0)
	  	        {
	  	            // Pure maximum power blast on boot up
	  	            final_ccr_target = MAX_ALLOWED_RAMP;
	  	            integral_i = 0.0f;
	  	            integral_v = 0.0f;
	  	            diff_i = 0.0f;
	  	            diff_v = 0.0f;
	  	          operating_mode = "BOOSTING CAP";
	  	          HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);

	  	        }
	  	        else
	  	        {
	  	            // 2. TRUE PI REGULATION LOOP
	  	            // Standard Forward Logic: Target - Current


	  	            if (duty_request_voltage < duty_request_current){
	  	                final_ccr_target = duty_request_voltage;
	  	            	operating_mode = "CV Mode";
	  	            	HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);
	  	            }
	  	            else{
	  	                final_ccr_target = duty_request_current;
	  	            	operating_mode = "CC Mode";
	  	            	HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_SET);
	  	            }
	  	        }
	  	      	  	      }


	  	        // =========================================================
	  	        // 3. YOUR HYSTERETIC SAFETY BRAKE (Placed after all math)
	  	        // =========================================================
	  	        // If the voltage is physically above target, bypass the loops and slam the brake!

	  	      /*if (current_v_reading > (target_voltage_adc + 200))
	  	            {
	  	                final_ccr_target = PWM_PERIOD_ARR - duty_request_voltage;
	  	               integral_v = 0.0f;
	  	                integral_i = 0.0f;
	  	            }*/
	  	        // 4. ABSOLUTE HARDWARE BOUNDARY CLAMPS
	  	        if (final_ccr_target > 1595) final_ccr_target = 1595;
	  	        if (final_ccr_target < 0)   final_ccr_target = 0;

	  	        // 5. HARDWARE REGISTER WRITE
	  	        TIM2->CCR1 = final_ccr_target;
	  	        TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

	  	      static uint32_t last_print = 0;
	  	            if (HAL_GetTick() - last_print > 100)
	  	            {
	  	                last_print = HAL_GetTick();
	  	                sprintf(tx_buffer, "\033[H\033[J=== AUTOMATIC CV/CC MONITOR ===\r\n"
	  	                                   "MODE STATUS : %s\r\n"
	  	                                   "Voltage ADC : %4lu\r\n"
	  	                                   "Current ADC : %4lu\r\n",
	  	                                   operating_mode,
	  	                                   current_v_reading,
	  	                                   current_i_reading);
	  	                HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer), 80);

	  	            }

	  	        HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 1679;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CC_LED_Pin */
  GPIO_InitStruct.Pin = CC_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CC_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
