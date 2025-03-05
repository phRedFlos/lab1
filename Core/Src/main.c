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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd1602_i2c.h"
#include <string.h>
//#include "DHT.h"
#include <stdio.h>  // Для sprintf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_5
#define BUFFER_SIZE 200 // 10 хвилин по 3 секунди

float tempBuffer[BUFFER_SIZE];
float humBuffer[BUFFER_SIZE];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SemaphoreHandle_t btnSemaphore;
uint32_t pMillis, cMillis;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

osThreadId SensorTaskHandle;
osThreadId ButtonTaskHandle;
osThreadId DisplayTaskHandle;
osMessageQId myQueueTempHandle;
osMessageQId myQueueHumHandle;
osMessageQId myQueueModeHandle;
/* USER CODE BEGIN PV */
lcd1602_HandleTypeDef lcd1602_Handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
void StartSensorTask(void const * argument);
void StartButtonTask(void const * argument);
void StartDisplayTask(void const * argument);

/* USER CODE BEGIN PFP */
void microDelay (uint16_t delay);
uint8_t DHT22_Start (void);
uint8_t DHT22_Read (void);
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
	xTraceEnable(TRC_START);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */
	lcd1602_Init(&lcd1602_Handle, &hi2c1, PCF8574_ADDRESS);
	HAL_TIM_Base_Start(&htim6);
	//	HAL_Delay(2000);
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	btnSemaphore = xSemaphoreCreateBinary();
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of myQueueTemp */
	osMessageQDef(myQueueTemp, 1, float);
	myQueueTempHandle = osMessageCreate(osMessageQ(myQueueTemp), NULL);

	/* definition and creation of myQueueHum */
	osMessageQDef(myQueueHum, 1, float);
	myQueueHumHandle = osMessageCreate(osMessageQ(myQueueHum), NULL);

	/* definition and creation of myQueueMode */
	osMessageQDef(myQueueMode, 1, uint8_t);
	myQueueModeHandle = osMessageCreate(osMessageQ(myQueueMode), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of SensorTask */
	osThreadDef(SensorTask, StartSensorTask, osPriorityNormal, 0, 256);
	SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

	/* definition and creation of ButtonTask */
	osThreadDef(ButtonTask, StartButtonTask, osPriorityNormal, 0, 128);
	ButtonTaskHandle = osThreadCreate(osThread(ButtonTask), NULL);

	/* definition and creation of DisplayTask */
	osThreadDef(DisplayTask, StartDisplayTask, osPriorityNormal, 0, 256);
	DisplayTaskHandle = osThreadCreate(osThread(DisplayTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
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
	hi2c1.Init.Timing = 0x00B07CB4;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 31;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 65535;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);
	xSemaphoreGiveFromISR(btnSemaphore, NULL);

}


void microDelay (uint16_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while (__HAL_TIM_GET_COUNTER(&htim6) < delay);
}

uint8_t DHT22_Start (void)
{
	uint8_t Response = 0;
	GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
	GPIO_InitStructPrivate.Pin = DHT22_PIN;
	GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as output
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
	microDelay (1300);   // wait for 1300us
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
	microDelay (30);   // wait for 30us
	GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as input
	microDelay (40);
	if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
	{
		microDelay (80);
		if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;
	}
	pMillis = xTaskGetTickCount();
	cMillis = xTaskGetTickCount();
	while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
	{
		cMillis = xTaskGetTickCount();
	}
	return Response;
}

uint8_t DHT22_Read (void)
{
	uint8_t a, b;
	for (a = 0; a < 8; a++)
	{
		pMillis = xTaskGetTickCount();
		cMillis = xTaskGetTickCount();
		while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
		{  // wait for the pin to go high
			cMillis = xTaskGetTickCount();
		}
		microDelay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
			b&= ~(1<<(7-a));
		else
			b|= (1<<(7-a));
		pMillis = xTaskGetTickCount();
		cMillis = xTaskGetTickCount();
		while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
		{  // wait for the pin to go low
			cMillis = xTaskGetTickCount();
		}
	}
	return b;
}


float calculateAverage(float *buffer, uint16_t size)
{
	float sum = 0;
	for (uint16_t i = 0; i < size; i++)
	{
		sum += buffer[i];
	}
	return sum / size;
}

float findMax(float *buffer, uint16_t size)
{
	float max = buffer[0];
	for (uint16_t i = 1; i < size; i++)
	{
		if (buffer[i] > max)
		{
			max = buffer[i];
		}
	}
	return max;
}

float findMin(float *buffer, uint16_t size)
{
	float min = buffer[0];
	for (uint16_t i = 1; i < size; i++)
	{
		if (buffer[i] < min)
		{
			min = buffer[i];
		}
	}
	return min;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorTask */
/**
 * @brief  Function implementing the SensorTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
	/* USER CODE BEGIN 5 */
	uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
	float tCelsius = 0;
	float RH = 0;

	/* Infinite loop */
	for(;;)
	{
		if(DHT22_Start())
		{
			RH1 = DHT22_Read(); // First 8bits of humidity
			RH2 = DHT22_Read(); // Second 8bits of Relative humidity
			TC1 = DHT22_Read(); // First 8bits of Celsius
			TC2 = DHT22_Read(); // Second 8bits of Celsius
			SUM = DHT22_Read(); // Check sum
			CHECK = RH1 + RH2 + TC1 + TC2;
			if (CHECK == SUM)
			{
				if (TC1>127) // If TC1=10000000, negative temperature
				{
					tCelsius = (float)TC2/10*(-1);
				}
				else
				{
					tCelsius = (float)((TC1<<8)|TC2)/10;
				}

				RH = (float) ((RH1<<8)|RH2)/10;

				xQueueSend(myQueueTempHandle, &tCelsius, portMAX_DELAY);
				xQueueSend(myQueueHumHandle, &RH, portMAX_DELAY);

				osDelay(3000);
			}
		}
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
 * @brief Function implementing the ButtonTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void const * argument)
{
	/* USER CODE BEGIN StartButtonTask */
	uint8_t displayMode = 0;
	/* Infinite loop */
	for(;;)
	{
		if (xSemaphoreTake(btnSemaphore, portMAX_DELAY) == pdTRUE)
		{
			displayMode = (displayMode + 1) % 4;
			xQueueSend(myQueueModeHandle, &displayMode, 0);
			osDelay(500);
			xSemaphoreTake(btnSemaphore, 0);
		}
		osDelay(1);
	}
	/* USER CODE END StartButtonTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
 * @brief Function implementing the DisplayTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
	/* USER CODE BEGIN StartDisplayTask */
	char lcdBuffer[50];
	float temperature;
	float hum;

	uint16_t bufferIndex = 0;
	uint16_t validDataCount = 0;
	uint8_t displayMode = 0;

	/* Infinite loop */
	for(;;)
	{
		xQueueReceive(myQueueTempHandle, &temperature, portMAX_DELAY);
		xQueueReceive(myQueueHumHandle, &hum, portMAX_DELAY);
		xQueueReceive(myQueueModeHandle, &displayMode, 0);

		tempBuffer[bufferIndex] = temperature;
		humBuffer[bufferIndex] = hum;
		bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

		if (validDataCount < BUFFER_SIZE)
		{
			validDataCount++;
		}

		lcd1602_Clear(&lcd1602_Handle);
		lcd1602_SetCursor(&lcd1602_Handle, 0, 0);

		switch (displayMode)
		{
		case 0:
			lcd1602_Print(&lcd1602_Handle, "Current:");
			sprintf(lcdBuffer, "T:%.1fC H:%.1f%%", temperature, hum);
			break;

		case 1:
		{
			float avgTemp = calculateAverage(tempBuffer, validDataCount);
			float avgHum = calculateAverage(humBuffer, validDataCount);
			lcd1602_Print(&lcd1602_Handle, "Average:");
			sprintf(lcdBuffer, "T:%.1fC H:%.1f%%", avgTemp, avgHum);
		}
		break;

		case 2:
		{
			float maxTemp = findMax(tempBuffer, validDataCount);
			float maxHum = findMax(humBuffer, validDataCount);
			lcd1602_Print(&lcd1602_Handle, "Maximum:");
			sprintf(lcdBuffer, "T:%.1fC H:%.1f%%", maxTemp, maxHum);
		}
		break;

		case 3:
		{
			float minTemp = findMin(tempBuffer, validDataCount);
			float minHum = findMin(humBuffer, validDataCount);
			lcd1602_Print(&lcd1602_Handle, "Minimum:");
			sprintf(lcdBuffer, "T:%.1fC H:%.1f%%", minTemp, minHum);
		}
		break;

		default:
			break;
		}

		lcd1602_SetCursor(&lcd1602_Handle, 0, 1);
		lcd1602_Print(&lcd1602_Handle, lcdBuffer);
		osDelay(3000);

		osDelay(1);
	}
	/* USER CODE END StartDisplayTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM16 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM16) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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
