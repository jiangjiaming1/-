/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for M3508control */
osThreadId_t M3508controlHandle;
const osThreadAttr_t M3508control_attributes = {
  .name = "M3508control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTest */
osThreadId_t myTestHandle;
const osThreadAttr_t myTest_attributes = {
  .name = "myTest",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for M8010control */
osThreadId_t M8010controlHandle;
const osThreadAttr_t M8010control_attributes = {
  .name = "M8010control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SBUS */
osThreadId_t SBUSHandle;
const osThreadAttr_t SBUS_attributes = {
  .name = "SBUS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GM6020control */
osThreadId_t GM6020controlHandle;
const osThreadAttr_t GM6020control_attributes = {
  .name = "GM6020control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Speed */
osThreadId_t SpeedHandle;
const osThreadAttr_t Speed_attributes = {
  .name = "Speed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void M3508controlTask(void *argument);
void myTestTask(void *argument);
void M8010controlTask(void *argument);
void SBUSTask(void *argument);
void GM6020controltask(void *argument);
void Speed_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of M3508control */
  M3508controlHandle = osThreadNew(M3508controlTask, NULL, &M3508control_attributes);

  /* creation of myTest */
  myTestHandle = osThreadNew(myTestTask, NULL, &myTest_attributes);

  /* creation of M8010control */
  M8010controlHandle = osThreadNew(M8010controlTask, NULL, &M8010control_attributes);

  /* creation of SBUS */
  SBUSHandle = osThreadNew(SBUSTask, NULL, &SBUS_attributes);

  /* creation of GM6020control */
  GM6020controlHandle = osThreadNew(GM6020controltask, NULL, &GM6020control_attributes);

  /* creation of Speed */
  SpeedHandle = osThreadNew(Speed_Task, NULL, &Speed_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_M3508controlTask */
/**
 * @brief Function implementing the M3508control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_M3508controlTask */
void M3508controlTask(void *argument)
{
  /* USER CODE BEGIN M3508controlTask */

  /* Infinite loop */
  for (;;)
  {
    M3508_Motor_Position_Ctrl();
    M3508_Motor_Speed_Ctrl();
    M3508_Motor_Send_Current();
    M3508_Motor_Send_Current_h();
    osDelay(1);
  }
  /* USER CODE END M3508controlTask */
}

/* USER CODE BEGIN Header_myTestTask */
/**
 * @brief Function implementing the myTest thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_myTestTask */
void myTestTask(void *argument)
{
  /* USER CODE BEGIN myTestTask */
  /* Infinite loop */
  for (;;)
  {

    osDelay(1);
  }
  /* USER CODE END myTestTask */
}

/* USER CODE BEGIN Header_M8010controlTask */
/**
 * @brief Function implementing the M8010control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_M8010controlTask */
void M8010controlTask(void *argument)
{
  /* USER CODE BEGIN M8010controlTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(10);
  }
  /* USER CODE END M8010controlTask */
}

/* USER CODE BEGIN Header_SBUSTask */
/**
 * @brief Function implementing the SBUS thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SBUSTask */
void SBUSTask(void *argument)
{
  /* USER CODE BEGIN SBUSTask */
  // CCDCONCTROL();

  /* Infinite loop */
  for (;;)
  {
    if (SBUS.ConnectState > 0)
      SBUS.ConnectState = SBUS.ConnectState - 1;
    switch (SBUS_Analyse_Switch(SBUS.CH8))
    {
    case -1:
      break;
    case 0:
      Robot.Robot_Contorl_Mode = CONTROL_MANUAL;
      Robot.Robot_in_self.velocity_exp.Vx = ROBOT_SPEED_MAX * SBUS_Analyse_Percent(SBUS.CH9) * (SBUS_Analyse_Percent(SBUS.CH1) - 0.5) * 2;
      Robot.Robot_in_self.velocity_exp.Vy = ROBOT_SPEED_MAX * SBUS_Analyse_Percent(SBUS.CH9) * (SBUS_Analyse_Percent(SBUS.CH2) - 0.5) * 2;
      Robot.Robot_in_self.velocity_exp.Vw = ROBOT_SPEED_YAW_MAX * SBUS_Analyse_Percent(SBUS.CH9) * (SBUS_Analyse_Percent(SBUS.CH4) - 0.5) * 20;
      break;
    case 1:
      Robot.Robot_Contorl_Mode = CONTROL_WORLD;
      Robot.Robot_in_world.velocity_exp.Vx = ROBOT_SPEED_MAX * SBUS_Analyse_Percent(SBUS.CH9) * (SBUS_Analyse_Percent(SBUS.CH1) - 0.5) * 2;
      Robot.Robot_in_world.velocity_exp.Vy = ROBOT_SPEED_MAX * SBUS_Analyse_Percent(SBUS.CH9) * (SBUS_Analyse_Percent(SBUS.CH2) - 0.5) * 2;
      Robot.Robot_in_world.velocity_exp.Vw = ROBOT_SPEED_YAW_MAX * SBUS_Analyse_Percent(SBUS.CH9) * (SBUS_Analyse_Percent(SBUS.CH4) - 0.5) * 20;
      break;
    case 2:
      Robot.Robot_Contorl_Mode = CONTROL_AUTO;
      break;
    }
      osDelay(10);
  }
  /* USER CODE END SBUSTask */
}

/* USER CODE BEGIN Header_GM6020controltask */
/**
 * @brief Function implementing the GM6020control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GM6020controltask */
void GM6020controltask(void *argument)
{
  /* USER CODE BEGIN GM6020controltask */
  for(int i=1;i<=4;i++)
  {
    GM6020[i].Expvel=7500;
  }
  /* Infinite loop */
  for (;;)
  {
    GM6020_Motor_Position_Ctrl();
    GM6020_Motor_Speed_Ctrl();
    GM6020_Motor_Current_Ctrl();
    GM6020_Motor_Current_Ctrl_h();
    osDelay(1);
  }
  /* USER CODE END GM6020controltask */
}

/* USER CODE BEGIN Header_Speed_Task */
/**
 * @brief Function implementing the Speed thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Speed_Task */
void Speed_Task(void *argument)
{
  /* USER CODE BEGIN Speed_Task */
  /* Infinite loop */
  for (;;)
  {
    if ((Robot.Robot_Contorl_Mode == CONTROL_MANUAL) )
    {
      float Vx, Vy, Vw;
      Vx = Robot.Robot_in_self.velocity_exp.Vx;
      Vy = Robot.Robot_in_self.velocity_exp.Vy;
      Vw = Robot.Robot_in_self.velocity_exp.Vw;
      if (!Vx && !Vy && !Vw)
      {
        //Robot_Velocity_World_To_Self();
      }
      // Robot_Velocity_World_To_Self();
      Robot_Velocity_To_Wheel_Speed();
      Robot_Direction_To_Wheel_Direction();
      Wheel_Speed_To_Motor();
      Wheel_Direction_To_Motor();
    }
    if ((Robot.Robot_Contorl_Mode == CONTROL_WORLD))
    {
      Robot_Velocity_World_To_Self();
      Robot_Velocity_To_Wheel_Speed();
      Robot_Direction_To_Wheel_Direction();
      Wheel_Direction_To_Motor();
      Wheel_Speed_To_Motor();
    }
    if ((Robot.Robot_Contorl_Mode == CONTROL_AUTO))
    {
      Robot_Velocity_World_To_Self();
      Robot_Velocity_To_Wheel_Speed();
      Robot_Direction_To_Wheel_Direction();
      Wheel_Direction_To_Motor();
      Wheel_Speed_To_Motor();
    }
//      Robot_Velocity_To_Wheel_Speed();
//      Robot_Direction_To_Wheel_Direction();
//      Wheel_Direction_To_Motor();
//      Wheel_Speed_To_Motor(); 
    osDelay(1);
  }
  /* USER CODE END Speed_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

