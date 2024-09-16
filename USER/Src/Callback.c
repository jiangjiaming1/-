/**
 * @file Callback.c
 * @author 闂傚倸瀚▔娑㈡偤濞嗘劖缍囬柨鐕傛嫹 (2405973406@qq.com)
 * @brief hal闁圭厧鐡ㄩ幐鐑芥嚈閹达箑妫樻い鎾跺仦缁€鈧柣鐘差儏閸燁偊宕甸柆宥呮瀬闁规鍠楅悾閬嶆倵閸︻厼浠ф鐑囨嫹
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
#include "main.h"
#include "string.h"
#define rx_buf_size 256

/**
 * @brief
 *
 * @param hcan
 */
uint8_t crc = 0;
uint8_t crc8_ccitt(uint8_t *data, uint32_t length);
uint8_t crc8_ccitt(uint8_t *data, uint32_t length)
{
    uint8_t crc = 0;

    while (length--)
    {
        crc ^= *data++;

        for (uint8_t i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x07 : crc << 1;
    }

    return crc;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
    case 0x201:
    case 0x202:
    case 0x203:
    case 0x204:
        M3508_Analyze(rx_header.StdId, rx_data);
        break;
    case 0x205:
    case 0x206:
    case 0x207:
    case 0x208:
        GM6020_Analyze(rx_header.StdId, rx_data);
        break;
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
    case 0x205:
    case 0x206:
    case 0x207:
    case 0x208:
        GM6020_Analyze(rx_header.StdId, rx_data);
        break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart->Instance == USART1)
	{
          SBUS_Analyze();  
          HAL_UART_Receive_IT(&huart1, &SBUS_Receive_ch, 1); // 接收到一帧数据后再次开启空闲中断
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}