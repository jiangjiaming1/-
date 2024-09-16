/**
 * @file SBUS.c
 * @author 陶子辰 (2405973406@qq.com)
 * @brief SBUS接收
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
#include "SBUS.h"

/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            	变量定义
*********************************************************************************************************
*/

/// @brief SBUS状态
SBUS_CH SBUS;
/// @brief SBUS接收缓冲区
uint8_t SBUS_RX_BUF[SBUS_BUFFER_SIZE];
uint8_t SBUS_Receive[SBUS_Receive_length] = {0};
/// @brief SBUS接收值
uint8_t SBUS_Receive_ch = 0; // 接受值
/// @brief SBUS接收标志
uint8_t SBUS_Receive_flag = 0; // 接收标志
/// @brief SBUS接收状态机
uint8_t SBUS_Receive_step = 0; // 状态机
/// @brief SBUS接收计数
uint8_t SBUS_Receive_count = 0; // 计数
/// @brief SBUS发送数据
uint8_t SBUS_Data_send[36] = {0};
/// @brief SBUS发送数据
/*static union
{
  uint8_t data[32];
  uint16_t SBUS_Data[16];
} SUS_vaule;*/
/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/
/**
 * @brief 分析拨杆位置
 * @param SBUS_CHANNEL_VALUE SBUS通道值
 * @return 拨杆位置,0为上,1为中,2为下，-1为无效值
 */
int8_t SBUS_Analyse_Switch(int16_t SBUS_CHANNEL_VALUE)
{
    if (SBUS_CHANNEL_VALUE < 353 || SBUS_CHANNEL_VALUE > 1695)
        return -1;
    else if (SBUS_CHANNEL_VALUE >= 353 && SBUS_CHANNEL_VALUE <= 600)
        return 0;
    else if (SBUS_CHANNEL_VALUE >= 601 && SBUS_CHANNEL_VALUE <= 1100)
        return 1;
    else if (SBUS_CHANNEL_VALUE >= 1101 && SBUS_CHANNEL_VALUE <= 1695)
        return 2;
    else
        return -1;
}
/**
 * @brief 分析百分比位置
 * @param SBUS_CHANNEL_VALUE SBUS通道值
 * @return 百分比位置,0~1，-1为无效值
 */
float SBUS_Analyse_Percent(int16_t SBUS_CHANNEL_VALUE)
{
    if (SBUS_CHANNEL_VALUE < 353 || SBUS_CHANNEL_VALUE > 1695)
        return -1;
    else
        return (SBUS_CHANNEL_VALUE - 353) / 1342.0f;
}
/**
 * @brief  SBUS接受中断
 * @param  None
 * @retval None
 
*/
/**
 * @brief  SBUS接受与解码
 * @param  None
 * @retval None
 */

void SBUS_Analyze(void)
{
  switch (SBUS_Receive_step)
  {
  case 0:
    if (SBUS_Receive_ch == SBUS_FRAME_HEADER)
    {
      SBUS_Receive_step++;
    }
    else
    {
      SBUS_Receive_step = 0;
    }
    break;
  case 1:
    SBUS_RX_BUF[SBUS_Receive_count] = SBUS_Receive_ch;
    SBUS_Receive_count++;
    if (SBUS_Receive_count >= 23)
    {
      SBUS_Receive_count = 0;
      SBUS_Receive_step++;
    }

    break;
  case 2:
    if (SBUS_Receive_ch == SBUS_FRAME_FOOTER)
    {
      SBUS.CH1 = ((int16_t)SBUS_RX_BUF[0] >> 0 | ((int16_t)SBUS_RX_BUF[1] << 8)) & 0x07FF;
      SBUS.CH2 = ((int16_t)SBUS_RX_BUF[1] >> 3 | ((int16_t)SBUS_RX_BUF[2] << 5)) & 0x07FF;
      SBUS.CH3 = ((int16_t)SBUS_RX_BUF[2] >> 6 | ((int16_t)SBUS_RX_BUF[3] << 2) | (int16_t)SBUS_RX_BUF[4] << 10) & 0x07FF;
      SBUS.CH4 = ((int16_t)SBUS_RX_BUF[4] >> 1 | ((int16_t)SBUS_RX_BUF[5] << 7)) & 0x07FF;
      SBUS.CH5 = ((int16_t)SBUS_RX_BUF[5] >> 4 | ((int16_t)SBUS_RX_BUF[6] << 4)) & 0x07FF;
      SBUS.CH6 = ((int16_t)SBUS_RX_BUF[6] >> 7 | ((int16_t)SBUS_RX_BUF[7] << 1) | (int16_t)SBUS_RX_BUF[8] << 9) & 0x07FF;
      SBUS.CH7 = ((int16_t)SBUS_RX_BUF[8] >> 2 | ((int16_t)SBUS_RX_BUF[9] << 6)) & 0x07FF;
      SBUS.CH8 = ((int16_t)SBUS_RX_BUF[9] >> 5 | ((int16_t)SBUS_RX_BUF[10] << 3)) & 0x07FF;
      SBUS.CH9 = ((int16_t)SBUS_RX_BUF[11] >> 0 | ((int16_t)SBUS_RX_BUF[12] << 8)) & 0x07FF;
      SBUS.CH10 = ((int16_t)SBUS_RX_BUF[12] >> 3 | ((int16_t)SBUS_RX_BUF[13] << 5)) & 0x07FF;
      SBUS.CH11 = ((int16_t)SBUS_RX_BUF[13] >> 6 | ((int16_t)SBUS_RX_BUF[14] << 2) | (int16_t)SBUS_RX_BUF[15] << 10) & 0x07FF;
      SBUS.CH12 = ((int16_t)SBUS_RX_BUF[15] >> 1 | ((int16_t)SBUS_RX_BUF[16] << 7)) & 0x07FF;
      SBUS.CH13 = ((int16_t)SBUS_RX_BUF[16] >> 4 | ((int16_t)SBUS_RX_BUF[17] << 4)) & 0x07FF;
      SBUS.CH14 = ((int16_t)SBUS_RX_BUF[17] >> 7 | ((int16_t)SBUS_RX_BUF[18] << 1) | (int16_t)SBUS_RX_BUF[19] << 9) & 0x07FF;
      SBUS.CH15 = ((int16_t)SBUS_RX_BUF[19] >> 2 | ((int16_t)SBUS_RX_BUF[20] << 6)) & 0x07FF;
      SBUS.CH16 = ((int16_t)SBUS_RX_BUF[20] >> 5 | ((int16_t)SBUS_RX_BUF[21] << 3)) & 0x07FF;
      //SBUS_Send();
      if ((SBUS_RX_BUF[22] & 0x4) && (SBUS_RX_BUF[22] & 0x8))
      {
        SBUS.ConnectState = 0;
      }
      else
      {
        SBUS.ConnectState = 100;
      }

      SBUS_Receive_step = 0;
    }
    else
    {
      SBUS_Receive_step = 0;
    }
    break;
  }
}







/**
 * @brief  SBUS接受与解码
 * @param  None
 * @retval None
 */
/*void SBUS_Analyze(void)
{
    // uint8_t i=0;

    // if(SBUS_RX_BUF[i]==SBUS_FRAME_HEADER&&SBUS_RX_BUF[i+SBUS_FRAME_SIZE-1]==SBUS_FRAME_FOOTER)
    // {
    //     SBUS.CH1=((int16_t)SBUS_RX_BUF[i+1]>>0|((int16_t)SBUS_RX_BUF[i+2]<<8))&0x07FF;
    //     SBUS.CH2=((int16_t)SBUS_RX_BUF[i+2]>>3|((int16_t)SBUS_RX_BUF[i+3]<<5))&0x07FF;
    //     SBUS.CH3=((int16_t)SBUS_RX_BUF[i+3]>>6|((int16_t)SBUS_RX_BUF[i+4]<<2)|(int16_t)SBUS_RX_BUF[5]<<10)&0x07FF;
    //     SBUS.CH4=((int16_t)SBUS_RX_BUF[i+5]>>1|((int16_t)SBUS_RX_BUF[i+6]<<7))&0x07FF;
    //     SBUS.CH5=((int16_t)SBUS_RX_BUF[i+6]>>4|((int16_t)SBUS_RX_BUF[i+7]<<4))&0x07FF;
    //     SBUS.CH6=((int16_t)SBUS_RX_BUF[i+7]>>7|((int16_t)SBUS_RX_BUF[i+8]<<1)|(int16_t)SBUS_RX_BUF[9]<<9)&0x07FF;
    //     SBUS.CH7=((int16_t)SBUS_RX_BUF[i+9]>>2|((int16_t)SBUS_RX_BUF[i+10]<<6))&0x07FF;
    //     SBUS.CH8=((int16_t)SBUS_RX_BUF[i+10]>>5|((int16_t)SBUS_RX_BUF[i+11]<<3))&0x07FF;
    //     SBUS.CH9=((int16_t)SBUS_RX_BUF[i+12]>>0|((int16_t)SBUS_RX_BUF[i+13]<<8))&0x07FF;
    //     SBUS.CH10=((int16_t)SBUS_RX_BUF[i+13]>>3|((int16_t)SBUS_RX_BUF[i+14]<<5))&0x07FF;
    //     SBUS.CH11=((int16_t)SBUS_RX_BUF[i+14]>>6|((int16_t)SBUS_RX_BUF[i+15]<<2)|(int16_t)SBUS_RX_BUF[i+16]<<10)&0x07FF;
    //     SBUS.CH12=((int16_t)SBUS_RX_BUF[i+16]>>1|((int16_t)SBUS_RX_BUF[i+17]<<7))&0x07FF;
    //     SBUS.CH13=((int16_t)SBUS_RX_BUF[i+17]>>4|((int16_t)SBUS_RX_BUF[i+18]<<4))&0x07FF;
    //     SBUS.CH14=((int16_t)SBUS_RX_BUF[i+18]>>7|((int16_t)SBUS_RX_BUF[i+19]<<1)|(int16_t)SBUS_RX_BUF[i+20]<<9)&0x07FF;
    //     SBUS.CH15=((int16_t)SBUS_RX_BUF[i+20]>>2|((int16_t)SBUS_RX_BUF[i+21]<<6))&0x07FF;
    //     SBUS.CH16=((int16_t)SBUS_RX_BUF[i+21]>>5|((int16_t)SBUS_RX_BUF[i+22]<<3))&0x07FF;
    //     if((SBUS_RX_BUF[i+23]&0x4)&&(SBUS_RX_BUF[i+23]&0x8))
    //         SBUS.ConnectState=0;
    //     else
    //         SBUS.ConnectState=100;
    // }

    // SBUS.CH1=((int16_t)data[0]>>0|((int16_t)data[1]<<8))&0x07FF;
    // SBUS.CH2=((int16_t)data[1]>>3|((int16_t)data[2]<<5))&0x07FF;
    // SBUS.CH3=((int16_t)data[2]>>6|((int16_t)data[3]<<2)|(int16_t)data[4]<<10)&0x07FF;
    // SBUS.CH4=((int16_t)data[4]>>1|((int16_t)data[5]<<7))&0x07FF;
    // SBUS.CH5=((int16_t)data[5]>>4|((int16_t)data[6]<<4))&0x07FF;
    // SBUS.CH6=((int16_t)data[6]>>7|((int16_t)data[7]<<1)|(int16_t)data[8]<<9)&0x07FF;
    // SBUS.CH7=((int16_t)data[8]>>2|((int16_t)data[9]<<6))&0x07FF;
    // SBUS.CH8=((int16_t)data[9]>>5|((int16_t)data[10]<<3))&0x07FF;
    // SBUS.CH9=((int16_t)data[11]>>0|((int16_t)data[12]<<8))&0x07FF;
    // SBUS.CH10=((int16_t)data[12]>>3|((int16_t)data[13]<<5))&0x07FF;
    // SBUS.CH11=((int16_t)data[13]>>6|((int16_t)data[14]<<2)|(int16_t)data[15]<<10)&0x07FF;
    // SBUS.CH12=((int16_t)data[15]>>1|((int16_t)data[16]<<7))&0x07FF;
    // SBUS.CH13=((int16_t)data[16]>>4|((int16_t)data[17]<<4))&0x07FF;
    // SBUS.CH14=((int16_t)data[17]>>7|((int16_t)data[18]<<1)|(int16_t)data[19]<<9)&0x07FF;
    // SBUS.CH15=((int16_t)data[19]>>2|((int16_t)data[20]<<6))&0x07FF;
    // SBUS.CH16=((int16_t)data[20]>>5|((int16_t)data[21]<<3))&0x07FF;
}*/