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
#include "fdcan.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "command.h"
#include "ws2812.h"
#include "drv_can.h"
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
uint8_t receive[3];
uint8_t receive2[]="b";
uint8_t readBuffer[10];
uint8_t command[50];
int commandLength = 0;
volatile int mark=1;
volatile uint32_t can_rx_count        = 0;   /* ← 新增: 收到多少帧 */
volatile uint32_t can_rx_last_id      = 0;   /* ← 新增: 最近一帧的 ID */
volatile uint8_t  can_rx_last_data[8] = {0}; /* ← 新增: 最近一帧的数据 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief CAN1 接收回调
 * @note  在 FDCAN1_IT0 中断里被调用, 只做搬运, 不要 HAL_Delay
 * @param Header 帧头: Header->Identifier 就是帧 ID, Header->DataLength 是长度
 * @param Buffer 8 字节数据, 出了回调就失效, 需要就拷走
 */
static void CAN1_RxCpltCallback(FDCAN_RxHeaderTypeDef *Header, uint8_t *Buffer)
{
  can_rx_count++;
  can_rx_last_id = Header->Identifier;
  for (int i = 0; i < 8; i++) {
    can_rx_last_data[i] = Buffer[i];
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if (huart == &huart7){
    Command_Write(readBuffer, Size);

    commandLength = Command_GetCommand(command);
    if (commandLength != 0) {
      HAL_UART_Transmit_DMA(&huart7, (uint8_t *)command, commandLength);
      if (command[0] == '1') {
        mark = 1;
      }
      else if (command[0] == '0') {
        mark = 0;
      }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, readBuffer, sizeof(readBuffer));
    __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
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

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

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
  MX_SPI6_Init();
  MX_UART7_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, readBuffer, sizeof(readBuffer));
  __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
  init_finished = true;                      /* ← 新增, 必须放最后 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // /* 每 500ms 发一帧测试帧, 验证发送通路 */
    // static uint32_t last_tx = 0;
    // if (HAL_GetTick() - last_tx >= 500U) {
    //   last_tx = HAL_GetTick();
    //   uint8_t test_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    //   CAN_Transmit_Data(&hfdcan1, 0x123, test_data, 8);
    // }
    //
    //
    //
    // if (mark == 1) {
    //   //红->绿
    //   for (int i=0; i<=255; i++) {
    //     WS2812_Ctrl(255-i,i,0);
    //     HAL_Delay(1);
    //   }
    //   //绿->蓝
    //   for (int i=0; i<=255; i++) {
    //     WS2812_Ctrl(0,255-i,i);
    //     HAL_Delay(1);
    //   }
    //   //蓝->红
    //   for (int i=0; i<=255; i++) {
    //     WS2812_Ctrl(i,0,255-i);
    //     HAL_Delay(1);
    //   }
    // }
    // else if (mark == 0) {
    //   for (int i=0; i<2; i++) {
    //     WS2812_Ctrl(255,0,255);
    //     HAL_Delay(100);
    //     WS2812_Ctrl(255,255,255);
    //     HAL_Delay(200);
    //   }
    //   while (mark == 0) {
    //     // WS2812_Ctrl(255,0,0);
    //     WS2812_Ctrl(255,0,255);
    //   }
    // }

    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    // HAL_Delay(100);


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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
