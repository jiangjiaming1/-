/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define rx_buf_size  256     //最大接收字节数
//char RxBuffer[rx_buf_size];   //接收数据
//uint8_t aRxBuffer;			//接收中断缓冲
//uint8_t Uart6_Rx_Cnt = 0;
//uint8_t color=0;
//uint8_t r, g, b;
extern int loading;
extern int flag_color;
//int flag_color=0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles CAN2 TX interrupts.
  */
void CAN2_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_TX_IRQn 0 */

  /* USER CODE END CAN2_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_TX_IRQn 1 */

  /* USER CODE END CAN2_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
//int flag_color=0;
//void executeCommand(uint8_t r, uint8_t g, uint8_t b)
//{
//    // 根据颜色执行相应的命令
//
//    if (r > g && r > b && ((g + b) < r))
//    {
//        color=1;
//        if(loading == 0)
//        {
//        flag_color = 1;
//        loading = 1;
//        }
//        //红色
//       // HAL_UART_Transmit(&huart3, (uint8_t *)"执行红色命令\n", 18, 0xFFFF);
//    }
//    else if ( r > g && b > g )
//    {
//        // 紫色最强，执行紫色命令
//       color=2;
//       if(loading == 0)
//       {
//       flag_color = 2;
//       loading = 1;
//       }
//        //HAL_UART_Transmit(&huart3, (uint8_t *)"执行绿色命令\n", 20, 0xFFFF);
//    }
//    else if (b > r && b > g && (r + g) < b)
//    {
//      color=3;
//      if(loading == 0)
//      {
//      flag_color = 3;
//      loading = 1;
//      }
//         // 蓝色最强，执行蓝色命令
//        //HAL_UART_Transmit(&huart3, (uint8_t *)"执行蓝色命令\n", 19, 0xFFFF);
//    }
//    else
//    {
//       color=4;
//       if(loading == 0)
//       {
//       flag_color = 0;
//       }
//        //HAL_UART_Transmit(&huart3, (uint8_t *)"执行默认命令\n", 20, 0xFFFF);
//    }
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
// // HAL_UART_Receive_IT(&huart6, (uint8_t *)&aRxBuffer, 1); 
//  
//	if (huart->Instance == USART6)
//  {
//    
//
//    if (Uart6_Rx_Cnt >= 255) // 数据溢出
//    {
//      Uart6_Rx_Cnt = 0;
//      memset(RxBuffer, 0x00, sizeof(RxBuffer)); // 清空数组数据
//      //		HAL_UART_Transmit(&huart4, (uint8_t *)"数据溢出", 10,0xFFFF);
//    }
//    else
//    {
//      RxBuffer[Uart6_Rx_Cnt++] = aRxBuffer; //
//
//      if ((RxBuffer[Uart6_Rx_Cnt - 1] == 0x0A) && (RxBuffer[Uart6_Rx_Cnt - 2] == 0x0D)) //
//      {
//        // 提取RGB值并执行相应的命令
//
//        if (sscanf((char *)&RxBuffer, "O+COLOR= R:%hhu G:%hhu B:%hhu", &r, &g, &b) == 3)
//        {
//          executeCommand(r, g, b);
//        }
//        else
//        {
//          color = 132;
//          // 无效的RGB格式
//          //                HAL_UART_Transmit(&huart4, (uint8_t *)"无效的RGB格式", 18, 0xFFFF);
//        }
//        Uart6_Rx_Cnt = 0;
//        memset(RxBuffer, 0x00, sizeof(RxBuffer)); // 清空数组数据
//      }
//    }
//    HAL_UART_Receive_IT(&huart6, (uint8_t *)&aRxBuffer, 1);
//  }
//  if (huart->Instance == USART1)
//  {
//    SBUS_Analyze();  
//    HAL_UART_Receive_IT(&huart1, &SBUS_Receive_ch, 1); // 接收到一帧数据后再次开启空闲中断
//                                              // 解析数据
//  }
//        
//	  //
//}

/* USER CODE END 1 */
