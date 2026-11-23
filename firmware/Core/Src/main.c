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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "services/led_service.h"
#include "app/app.h"
#include "metrics/metrics.h"
#include "metrics/memory_metrics.h"
#include "metrics/metrics_config.h"
#if (METRICS_ENABLE_STARTUP_SELF_TEST != 0U)
#include "metrics/metrics_test.h"
volatile uint32_t g_metrics_test_result;
#endif
#include "FreeRTOS.h"
#include "task.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define APP_MAIN_TASK_STACK_SIZE    1024U
#define APP_MAIN_TASK_PRIORITY      (tskIDLE_PRIORITY + 1U)

#define LED_TASK_STACK_SIZE         256U
#define LED_TASK_PRIORITY           (tskIDLE_PRIORITY + 2U)
#define LED_TASK_PERIOD_MS          10U

static void AppMainTask(void *argument)
{
	(void)argument;

	for (;;) {
		Metrics_LoopBegin();
		App_RunOnce();
		Metrics_LoopEnd();
		Metrics_Poll();
	}
}

static void LedTask(void *argument)
{
	(void)argument;
	TickType_t last_wake = xTaskGetTickCount();

	for (;;) {
		Metrics_ExecBegin(METRICS_EXEC_TASK_LED, LED_TASK_PERIOD_MS);
		LedService_Update(App_GetState(), HAL_GetTick());
		Metrics_ExecEnd(METRICS_EXEC_TASK_LED);

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* Paint the reserved MSP stack before HAL/application initialization. */
	MemoryMetrics_StackPaint();

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
	/* Reserved measurement outputs: PA4, PB0, PB1 and PB10. */
	(void) Metrics_ProbeInit();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	/* Start application services after all hardware peripherals are initialized. */
	App_Init();
	/* Initialize metrics only after App_Init(): camera initialization may
	 * reset DWT->CYCCNT for its own microsecond delays. */
	if (!Metrics_Init()) {
		Error_Handler();
	}
#if (METRICS_ENABLE_STARTUP_SELF_TEST != 0U)
	g_metrics_test_result = MetricsTest_Run();
#endif
	/* USER CODE END 2 */

	/* USER CODE BEGIN WHILE */
	if (xTaskCreate(
		AppMainTask,
		"AppMain",
		APP_MAIN_TASK_STACK_SIZE,
		NULL,
		APP_MAIN_TASK_PRIORITY,
		NULL) != pdPASS)
	{
		Error_Handler();
	}

	if (xTaskCreate(
		LedTask,
		"LedTask",
		LED_TASK_STACK_SIZE,
		NULL,
		LED_TASK_PRIORITY,
		NULL) != pdPASS)
	{
		Error_Handler();
	}

	vTaskStartScheduler();

	Error_Handler();
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* The 8 MHz external clock and PLL settings below produce a 100 MHz SYSCLK. */

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* Stop execution if a required HAL peripheral initialization fails. */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* Inspect file and line in the debugger when an assertion fails. */
	(void) file;
	(void) line;
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
