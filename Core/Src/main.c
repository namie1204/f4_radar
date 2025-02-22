/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "CLCD_I2C.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CLCD_I2C_Name LCD1;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOF 0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define ALL_DATA 0x0A13
#define RESESPIRATORY_RATE 0x0A14
#define HEART_BEAT 0x0A15
#define DECTEC_DISTANCE 0x0A16

#define SOF_BYTE 0U
#define ID_BYTE1 1
#define ID_BYTE2 2

#define LEN_BYTE_H 3
#define LEN_BYTE_L 4

#define TYPE_BYTE_H 5
#define TYPE_BYTE_L 6

#define HEAD_CKS_BYTE 7

#define ALL_DATA_LEN 12
#define TW0_DATA_LEN 8
#define ONE_DATA_LEN 4

#define SHIFT_BIT_TYPE 8

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void ProcessRadarData(void);
void processFrame(uint8_t *buffer, uint16_t length);
static uint8_t getCksum(uint8_t *data, uint8_t len);
float bytesToFloat(uint8_t *data);
void OutputResult(const char *msg);
void LCD_Print(const char *msg);
/* USER CODE END PFP */
static float breath_rate;
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float get_breath_rate(void)
{
  return breath_rate;
}

static uint8_t getCksum(uint8_t *data, uint8_t len)
{
  uint8_t ret = 0;
  for (uint8_t i = 0; i < len; i++)
  {
    ret ^= data[i];
  }
  ret = ~ret;
  return ret;
}

float bytesToFloat(uint8_t *data)
{
  float f;
  memcpy(&f, data, sizeof(float));
  return f;
}

void LCD_Print(const char *msg)
{
  CLCD_I2C_SetCursor(&LCD1, 0, 1);
  CLCD_I2C_WriteString(&LCD1, (char *)msg);
}

void OutputResult(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void processFrame(uint8_t *buffer, uint16_t length)
{
  if (length < 8)
    return;

  if (buffer[0] != SOF)
    return;

  uint16_t id = (buffer[1] << 8) | buffer[2];
  uint16_t lenField = (buffer[3] << 8) | buffer[4];
  uint16_t type = (buffer[5] << 8) | buffer[6];
  uint8_t headCks = buffer[7];

  if (getCksum(buffer, 7) != headCks)
  {
    OutputResult("Header checksum error\r\n");
    return;
  }
  switch (type)
  {
  case 0x0A13:
  {
    if (length < 8 + 12 + 1)
      return;

    uint8_t *dataPtr = &buffer[8];
    uint8_t dataCks = buffer[8 + ALL_DATA_LEN];

    if (getCksum(dataPtr, 12) != dataCks)
    {
      OutputResult("Data checksum error (Phase Test)\r\n");
      return;
    }
    float total_phase = bytesToFloat(&dataPtr[0]);
    float breath_phase = bytesToFloat(&dataPtr[4]);
    float heart_phase = bytesToFloat(&dataPtr[8]);
    char msg[64];
    snprintf(msg, sizeof(msg), "Phase: Total=%f, Breath=%f, Heart=%f\r\n", total_phase, breath_phase, heart_phase);
    OutputResult(msg);
    break;
  }
  case 0x0A14:
  {
    if (length < 8 + 4 + 1)
      return;
    uint8_t *dataPtr = &buffer[8];
    uint8_t dataCks = buffer[8 + ONE_DATA_LEN];
    if (getCksum(dataPtr, 4) != dataCks)
      breath_rate
      {
        OutputResult("Data checksum error (Breath Rate)\r\n");
        return;
      }
    float = bytesToFloat(dataPtr);
    char msg[32];
    snprintf(msg, sizeof(msg), "Breath Rate: %f\r\n", breath_rate);
    OutputResult(msg);
    break;
  }
  case 0x0A15:
  {
    if (length < 8 + 4 + 1)
      return;
    uint8_t *dataPtr = &buffer[8];
    uint8_t dataCks = buffer[8 + 4];
    if (getCksum(dataPtr, 4) != dataCks)
    {
      OutputResult("Data checksum error (Heart Rate)\r\n");
      return;
    }
    float heart_rate = bytesToFloat(dataPtr);
    char msg[32];
    snprintf(msg, sizeof(msg), "Heart Rate: %f\r\n", heart_rate);
    OutputResult(msg);
    break;
  }
  case 0x0A16:
  {
    if (length < 8 + 8 + 1)
      return;
    uint8_t *dataPtr = &buffer[8];
    uint8_t dataCks = buffer[8 + 8];
    if (getCksum(dataPtr, 8) != dataCks)
    {
      OutputResult("Data checksum error (Target Distance)\r\n");
      return;
    }
    uint32_t flag = dataPtr[0] | (dataPtr[1] << 8) | (dataPtr[2] << 16) | (dataPtr[3] << 24);
    float range = bytesToFloat(&dataPtr[4]);
    if (flag == 1)
    {
      char msg[32];
      snprintf(msg, sizeof(msg), "Target Range: %f cm\r\n", range);
      OutputResult(msg);
    }
    else
    {
      OutputResult("No target distance output.\r\n");
    }
    break;
  }
  default:
  {
    char msg[32];
    snprintf(msg, sizeof(msg), "Unknown frame type: 0x%04X\r\n", type);
    OutputResult(msg);
    break;
  }
  }
}

void ProcessRadarData(void)
{
  uint8_t header[8];
  int dataLen = 0;
  int totalFrameSize = 0;
  ` uint8_t frameBuffer[64] = {0};

  if (HAL_UART_Receive(&huart1, header, 8, 100) == HAL_OK)
  {
    /*Check data invalid*/
    if (header[0] != SOF)
      return;

    /* read type*/
    uint16_t type = (header[TYPE_BYTE_H] << SHIFT_BIT_TYPE) | header[TYPE_BYTE_L];

    /* cal data length */
    switch (type)
    {
    case 0x0A13:
      dataLen = 12;
      break;
    case 0x0A14:
      dataLen = 4;
      break;
    case 0x0A15:
      dataLen = 4;
      break;
    case 0x0A16:
      dataLen = 8;
      break;
    default:
      return;
    }

    totalFrameSize = 8 + dataLen + 1;

    /* copy data from header to frame_buf with size is 8*/
    memcpy(frameBuffer, header, 8);

    if (HAL_UART_Receive(&huart1, &frameBuffer[8], dataLen + 1, 100) == HAL_OK)
    {
      processFrame(frameBuffer, totalFrameSize);
    }
  }
}
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  CLCD_I2C_Init(&LCD1, &hi2c1, 0x4e, 16, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ProcessRadarData();
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
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
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
  huart1.Init.BaudRate = 1382400;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

#ifdef USE_FULL_ASSERT
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
