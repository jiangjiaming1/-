/**
 * @file PID.c
 * @author 陶子辰 (2405973406@qq.com)
 * @brief PID闭环控制
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
#include "PID.h"

/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/
/**
 * @brief  增量式PID
 * @param  PIDptr: PID结构体指针
 * @param  ThisError: 当前误差
 * @retval PID计算结果
 */
float PIDCal ( PIDType *PIDptr, float ThisError ) // 增量式PID
{
  float pError,dError,iError,temp,integral;	
  
  pError = ThisError - PIDptr->LastErr; 
  iError = ThisError;
  dError = ThisError - 2 * ( PIDptr->LastErr ) + PIDptr->PreErr;
  integral = PIDptr->KI * iError;
  
  if( ( integral > PIDptr->KIlimit ) && ( PIDptr->KIlimit != 0 ) )
    integral = PIDptr->KIlimit;
  
  else if( ( integral < -PIDptr->KIlimit ) && ( PIDptr->KIlimit != 0 ) )
    integral = -PIDptr->KIlimit;
  
  if( ( ThisError > PIDptr->delErr ) || ( ThisError < -PIDptr->delErr ) )	
    temp = PIDptr->KP * pError + integral + PIDptr->KD * dError;  
  
  else
    temp = 0;
  
  PIDptr->PreErr = PIDptr->LastErr;
  PIDptr->LastErr = ThisError;
  
  return temp;
}
