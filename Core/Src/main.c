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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "lis2dw12_reg.h"
#include "stm32f7xx_hal.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

extern UART_HandleTypeDef huart1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
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
	  /* Initialize mems driver interface */
	  stmdev_ctx_t dev_ctx;
	  dev_ctx.write_reg = platform_write;
	  dev_ctx.read_reg = platform_read;
	  dev_ctx.mdelay = platform_delay;
	  dev_ctx.handle = &SENSOR_BUS;

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize platform specific hardware */

  /* Check device ID */
  /* Initialize platform specific hardware */

  /* Wait sensor boot time */
  platform_delay(250);
  /* Check device ID */
  lis2dw12_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LIS2DW12_ID)
    while (1) {
      /* manage here device not found */
    }

  /* Restore default configuration */
  lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lis2dw12_reset_get(&dev_ctx, &rst);
  } while (rst);


  lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set full scale */
    //lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_8g);
    lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);
    /* Configure filtering chain
     * Accelerometer - filter path / bandwidth
     */
    lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
    lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);
    /* Configure power mode */
    lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
    //lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit);
    /* Set Output Data Rate */
    lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_25Hz);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    char chaine[250];
  HAL_GPIO_WritePin(user_le_GPIO_Port, user_le_Pin, GPIO_PIN_RESET);
  printf("Projet Lancé\r\n");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint8_t reg;
	      /* Read output only if new value is available */
	      lis2dw12_flag_data_ready_get(&dev_ctx, &reg);

	      if (reg) {
	        /* Read acceleration data */
	        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	        lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
	        //acceleration_mg[0] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[0]);
	        //acceleration_mg[1] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[1]);
	        //acceleration_mg[2] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[2]);
	        acceleration_mg[0] = lis2dw12_from_fs2_to_mg(
	                               data_raw_acceleration[0]);
	        acceleration_mg[1] = lis2dw12_from_fs2_to_mg(
	                               data_raw_acceleration[1]);
	        acceleration_mg[2] = lis2dw12_from_fs2_to_mg(
	                               data_raw_acceleration[2]);
	        snprintf((char *)tx_buffer, sizeof(tx_buffer),
	                "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
	                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
	        printf(tx_buffer);
	        HAL_GPIO_TogglePin(user_le_GPIO_Port, user_le_Pin);
	      }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}
/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
HAL_Delay(ms);
}
///////////////////////////////////////////////////////////////////////
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);  // pour pouvoir afficher les print dans putty
    return ch;
}
////////////////////////////////////////////////////////////////////
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
