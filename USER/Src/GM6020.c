/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include "GM6020.h"

/*
*********************************************************************************************************
*                                            	��������
*********************************************************************************************************
*/
uint8_t jishu2 = 0;

GM6020_MOTOR GM6020[9];
// �ٶȻ�PID
PIDType GM6020SpeedPID1 = {7, 0.7, 0, 0, 0, 0, 0};
PIDType GM6020SpeedPID2 = {7, 0.7, 0, 0, 0, 0, 0};
PIDType GM6020SpeedPID3 = {7, 0.7, 0, 0, 0, 0, 0};
PIDType GM6020SpeedPID4 = {7, 0.7, 0, 0, 0, 0, 0};
PIDType GM6020SpeedPID5 = {7, 12, 10.3, 0, 0, 0, 0};
PIDType GM6020SpeedPID6 = {13, 0.72, 0, 0, 0, 0, 0};
PIDType GM6020SpeedPID7 = {7, 0.7, 0, 0, 0, 0, 0};
PIDType GM6020SpeedPID8 = {7, 0.7, 0, 0, 0, 0, 0};
PIDType Motor5PID = {17.5, 0.85, 0, 0, 0, 0, 0};
PIDType Motor6PID = {17.5, 0.85, 0, 0, 0, 0, 0};
PIDType Motor7PID = {17.5, 0.85, 0, 0, 0, 0, 0};
PIDType Motor8PID = {17.5, 0.85, 0, 0, 0, 0, 0};

// λ�û�PID
PIDType GM6020PositionPID1 = {0.10, 0.10, 0, 0, 0, 0, 0};
PIDType GM6020PositionPID2 = {0.10, 0.10, 0, 0, 0, 0, 0};
PIDType GM6020PositionPID3 = {0.10, 0.10, 0, 0, 0, 0, 0};
PIDType GM6020PositionPID4 = {0.10, 0.10, 0, 0, 0, 0, 0};
PIDType GM6020PositionPID5 = {0.1, 0.05, 0, 0, 0, 0, 0};
PIDType GM6020PositionPID6 = {0.2, 0.10, 0, 0, 0, 0, 0};
PIDType GM6020PositionPID7 = {0.05, 0.04, 0.03, 0, 0, 0, 0};
PIDType GM6020PositionPID8 = {0.10, 0.10, 0, 0, 0, 0, 0};

/**
 * @brief ����C620�������������.
 *
 * @param[in] canmsg: CAN��Ϣָ��
 *            canmsg->StdId: CAN��Ϣ��׼ID
 *            canmsg->Data: CAN��Ϣ����
 *            canmsg->Data[0]:ת�ӻ�е�Ƕȸ�8λ
 *            canmsg->Data[1]:ת�ӻ�е�Ƕȵ�8λ
 *            canmsg->Data[2]:ת��ת�ٸ�8λ
 *            canmsg->Data[3]:ת��ת�ٵ�8λ
 *            canmsg->Data[4]:ʵ��ת�ص�����8λ
 *            canmsg->Data[5]:ʵ��ת�ص�����8λ
 *            canmsg->Data[6]:����¶�
 *            canmsg->Data[7]:NULL
 *            canmsg->DLC: CAN��Ϣ���ݳ���
 *
 * @return none
 *
 * @note �ú������ڷ���C620������������ݣ�����ת�ӻ�е�Ƕȡ�ת�١��������¶ȵ���Ϣ��
 *       �������canmsg����CAN��Ϣ��׼ID�����ݺ����ݳ��ȡ�
 *       �ú����������κ�ֵ��
 *       ����Ƶ�ʣ�1KHz
 *       ת�ӻ�е�Ƕ�ֵ��Χ��0~8191����Ӧ0~360�㣩
 *       ת��ת��ֵ��λ��RPM
 *       ����¶ȵ�λ����
 *
 * @example
 *         CanRxMsg canmsg;
 *         Motor_Analyze(&canmsg);
 */
void GM6020_Analyze(uint32_t GM6020_StdId, uint8_t *MotorData)
{

  char Motor_Id;
  int16_t MotorVal[9][3];

  Motor_Id = GM6020_StdId - 0x204;

  MotorVal[Motor_Id][0] = MotorData[0] << 8 | MotorData[1]; // ת�ӻ�е�Ƕ�
  MotorVal[Motor_Id][1] = MotorData[2] << 8 | MotorData[3]; // ת��ת��

  MotorVal[Motor_Id][2] = MotorData[4] << 8 | MotorData[5]; // ת�ص���

  GM6020[Motor_Id].posdel = MotorVal[Motor_Id][0] - GM6020[Motor_Id].CurPosition;

  if (GM6020[Motor_Id].CurPosition == 0)
  {
    GM6020[Motor_Id].posdel = 0;
  }

  if (GM6020[Motor_Id].posdel >= 4095 && GM6020[Motor_Id].CurPosition != 0)
  {
    GM6020[Motor_Id].posdel = GM6020[Motor_Id].posdel - 8192;
  }
  if (GM6020[Motor_Id].posdel <= -4095 && GM6020[Motor_Id].CurPosition != 0)
  {
    GM6020[Motor_Id].posdel = GM6020[Motor_Id].posdel + 8192;
  }

  GM6020[Motor_Id].Temparg += GM6020[Motor_Id].posdel;
  GM6020[Motor_Id].CurPosition = MotorVal[Motor_Id][0];
  GM6020[Motor_Id].CurSpeed = MotorVal[Motor_Id][1];
  GM6020[Motor_Id].ActCurrent = MotorVal[Motor_Id][2];
  GM6020[Motor_Id].absolute_temp = MotorVal[Motor_Id][0];
}

/**
 * @brief ������ƺ���.
 *
 * @param[in] none
 *
 * @return none
 *
 * @note �ú�������ʵ��GM6020���ٶȱջ����ơ�
 *
 * @example
 *         GM6020_Motor_Speed_Ctrl();
 */
void GM6020_Motor_Speed_Ctrl(void)
{
  GM6020[1].OutCurrent += PIDCal(&GM6020SpeedPID1,GM6020[1].ExpSpeed - GM6020[1].CurSpeed);
  GM6020[2].OutCurrent += PIDCal(&GM6020SpeedPID2,GM6020[2].ExpSpeed - GM6020[2].CurSpeed);
  GM6020[3].OutCurrent += PIDCal(&GM6020SpeedPID3,GM6020[3].ExpSpeed - GM6020[3].CurSpeed);
  GM6020[4].OutCurrent += PIDCal(&GM6020SpeedPID4,GM6020[4].ExpSpeed - GM6020[4].CurSpeed);
  GM6020[5].OutCurrent += PIDCal(&GM6020SpeedPID5,GM6020[5].ExpSpeed - GM6020[5].CurSpeed);
  GM6020[6].OutCurrent += PIDCal(&GM6020SpeedPID6,GM6020[6].ExpSpeed - GM6020[6].CurSpeed);
  GM6020[7].OutCurrent += PIDCal(&GM6020SpeedPID7,GM6020[7].ExpSpeed - GM6020[7].CurSpeed);
  GM6020[8].OutCurrent += PIDCal(&GM6020SpeedPID8,GM6020[8].ExpSpeed - GM6020[8].CurSpeed);
}
/**
 * @brief ������ƺ���.
 *
 * @param[in] GM6020_id ���ID
 * @param[in] GM6020PositionPIDx ���λ��PID
 *
 * @return GM6020ExpSpeed ��������ٶ�
 *
 * @note �ú�������ʵ�ֵ���GM6020��λ�ñջ����ơ�
 *
 * @example
 *         GM6020_Motor_Signal_Position_Ctrl();
 */
float GM6020_Motor_Signal_Position_Ctrl(int GM6020_id, PIDType *GM6020PositionPIDx)
{
  float GM6020ExpSpeed;
  int32_t now = GM6020[GM6020_id].absolute_temp; // GM6020[GM6020_id].Exparg;
  if (GM6020[GM6020_id].Exparg - GM6020[GM6020_id].absolute_temp > 4096)
  {
    now += 8192;
  }
  else if (GM6020[GM6020_id].Exparg - GM6020[GM6020_id].absolute_temp < -4096)
  {
    now -= 8192;
  }
  GM6020ExpSpeed = PIDCal(GM6020PositionPIDx, GM6020[GM6020_id].Exparg - now);
  if (GM6020ExpSpeed > GM6020[GM6020_id].Expvel)
    GM6020ExpSpeed = GM6020[GM6020_id].Expvel;
  else if (GM6020ExpSpeed < -GM6020[GM6020_id].Expvel)
    GM6020ExpSpeed = -GM6020[GM6020_id].Expvel;
  return GM6020ExpSpeed;

  /*if (abs(GM6020[GM6020_id].Exparg - GM6020[GM6020_id].Temparg) < 11000)
  {
    if (PIDCal(GM6020PositionPIDx, GM6020[GM6020_id].Exparg - GM6020[GM6020_id].Temparg) < -5)
    {
      GM6020ExpSpeed = -5;
    }
    else if (PIDCal(GM6020PositionPIDx, GM6020[GM6020_id].Exparg - GM6020[GM6020_id].Temparg) > 5)
    {
      GM6020ExpSpeed = 5;
    }
    else
    {
      GM6020ExpSpeed = PIDCal(GM6020PositionPIDx, GM6020[GM6020_id].Exparg - GM6020[GM6020_id].Temparg);
    }
  }
  else if (GM6020[GM6020_id].Exparg > GM6020[GM6020_id].Temparg)
  {
    GM6020ExpSpeed = 1.05 * GM6020[GM6020_id].Expvel;
  }
  else
  {
    GM6020ExpSpeed = -1.05 * GM6020[GM6020_id].Expvel;
  }
  return GM6020ExpSpeed;*/
}
/**
 * @brief ������ƺ���.
 *
 * @param[in] none
 *
 * @return none
 *
 * @note �ú�������ʵ��GM6020��λ�ñջ����ơ�
 *
 * @example
 *         GM6020_Motor_Position_Ctrl();
 */
void GM6020_Motor_Position_Ctrl(void)
{
  GM6020[1].ExpSpeed = (int16_t)GM6020_Motor_Signal_Position_Ctrl(1, &GM6020PositionPID1);
  GM6020[2].ExpSpeed = (int16_t)GM6020_Motor_Signal_Position_Ctrl(2, &GM6020PositionPID2);
  GM6020[3].ExpSpeed = (int16_t)GM6020_Motor_Signal_Position_Ctrl(3, &GM6020PositionPID3);
  GM6020[4].ExpSpeed = (int16_t)GM6020_Motor_Signal_Position_Ctrl(4, &GM6020PositionPID4);
  GM6020[5].ExpSpeed = (int16_t)GM6020_Motor_Signal_Position_Ctrl(5, &GM6020PositionPID5);
  GM6020[6].ExpSpeed = (int16_t)GM6020_Motor_Signal_Position_Ctrl(6, &GM6020PositionPID6);
  GM6020[7].ExpSpeed = (int16_t)GM6020_Motor_Signal_Position_Ctrl(7, &GM6020PositionPID7);
  GM6020[8].ExpSpeed = (int16_t)GM6020_Motor_Signal_Position_Ctrl(8, &GM6020PositionPID8);
}

/**
 * @brief ��C620�������ָ����Ƶ���.
 *
 * @param[in] none
 *
 * @return none
 *
 * @note �ú���������C620�������ָ����Ƶ�����
 *       can_id=0x200�����ID��1~4����0x1FF�����ID��5~8��
 *       ���Ƶ���ֵ��Χ -16384~16384  ��Ӧת�ص�����Χ -20~20A
 *       data[0]:���Ƶ�����8λ
 *       data[1]:���Ƶ�����8λ
 *       data[2]:���Ƶ�����8λ
 *       data[3]:���Ƶ�����8λ
 *       data[4]:���Ƶ�����8λ
 *       data[5]:���Ƶ�����8λ
 *       data[6]:���Ƶ�����8λ
 *       data[7]:���Ƶ�����8λ
 *
 * @example
 *        GM6020_Motor_Current_Ctrl(0x200, 0, 0, 0, 0, 0, 0, 0, 0);
 */
void GM6020_Motor_Current_Ctrl(void)
{
  uint8_t i;
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.StdId = 0x1FF;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 0x08;
  TxHeader.TransmitGlobalTime = DISABLE;
  uint8_t Txmsg[8] = {0};

  for (i = 1; i <= 8; i++)
  {
    if (GM6020[i].OutCurrent > 16384)
    {
      GM6020[i].OutCurrent = 16384;
    }
    else if (GM6020[i].OutCurrent < -16384)
    {
      GM6020[i].OutCurrent = -16384;
    }
  }
  Txmsg[0] = (uint8_t)((GM6020[1].OutCurrent >> 8) & 0xFF);
  Txmsg[1] = (uint8_t)(GM6020[1].OutCurrent & 0xFF);
  Txmsg[2] = (uint8_t)((GM6020[2].OutCurrent >> 8) & 0xFF);
  Txmsg[3] = (uint8_t)(GM6020[2].OutCurrent & 0xFF);
  Txmsg[4] = (uint8_t)((GM6020[3].OutCurrent >> 8) & 0xFF);
  Txmsg[5] = (uint8_t)(GM6020[3].OutCurrent & 0xFF);
  Txmsg[6] = (uint8_t)((GM6020[4].OutCurrent >> 8) & 0xFF);
  Txmsg[7] = (uint8_t)(GM6020[4].OutCurrent & 0xFF);
  while (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Txmsg, &send_mail_box) != HAL_OK)
  {
    // Error_Handler();
  }
}
/**
 * @brief ������ƺ���.
 *
 * @param[in] none
 *
 * @return none
 *
 * @note �ú�������ʵ��GM6020�ĵ����ջ����ơ�
 *
 * @example
 *         GM6020_Motor_Current_Ctrl_h();
 */
void GM6020_Motor_Current_Ctrl_h(void)
{
  uint8_t i;
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.StdId = 0x2FF;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 0x08;
  TxHeader.TransmitGlobalTime = DISABLE;
  uint8_t Txmsg[8] = {0};

  for (i = 1; i <= 8; i++)
  {
    if (GM6020[i].OutCurrent > 16384)
    {
      GM6020[i].OutCurrent = 16384;
    }
    else if (GM6020[i].OutCurrent < -16384)
    {
      GM6020[i].OutCurrent = -16384;
    }
  }
  Txmsg[0] = (uint8_t)((GM6020[5].OutCurrent >> 8) & 0xFF);
  Txmsg[1] = (uint8_t)(GM6020[5].OutCurrent & 0xFF);
  Txmsg[2] = (uint8_t)((GM6020[6].OutCurrent >> 8) & 0xFF);
  Txmsg[3] = (uint8_t)(GM6020[6].OutCurrent & 0xFF);
  Txmsg[4] = (uint8_t)((GM6020[7].OutCurrent >> 8) & 0xFF);
  Txmsg[5] = (uint8_t)(GM6020[7].OutCurrent & 0xFF);
  Txmsg[6] = (uint8_t)((GM6020[8].OutCurrent >> 8) & 0xFF);
  Txmsg[7] = (uint8_t)(GM6020[8].OutCurrent & 0xFF);
  while (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Txmsg, &send_mail_box) != HAL_OK)
  {
    //        Error_Handler();
  }
}