/**
 * @file GO-M8010-6.h
 * @author 陶子辰 (2405973406@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-07-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "motor_control.h"

#include "stdio.h"
/*
*********************************************************************************************************
*                                             EXPORTED_TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
extern uint8_t step_Upper;
extern uint8_t Receive_from_Bellow_ch;
extern uint8_t Receive_from_Bellow_step;
extern uint8_t step_Bellow;
extern uint8_t inverse_flag;

HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData);
void Send_to_Bellow_step(void);

