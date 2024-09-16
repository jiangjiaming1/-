/**
 * @file RobotControl.c
 * @author jjm (2411241788@qq.com)
 * @brief 2024-底盘（运动解算+速度+路径规划）
 * @version
 * @date 2024-03-26
 *
 *
 */
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "main.h"
#include "math.h"
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/
const double robot_length = 0.650f;
const double robot_width = 0.650f;
const double Rotate_R = 0.32527;
#define Speed_Limit 2.0
#define Mutation_Limit 1.0
#define Speed_Limit_min 0.1
#define Initial_Speed_Limit 1.0
#define Add_Speed 0.1
#define Recify_Limit 0.3
#define Acceleration_Limit 0.7
#define Jerk_Limit 0.7
#define Distance_Error 0.2
#define Distance_Error_y 0.25
#define Distance_recify_Error 0.03
#define Circularity_Error 0.05
Robot_INFO Robot;
Position Original_Position_Y = {0, 0, 0};
Position Original_Position_X = {0, 0, 0};
Position Original_Position_XY = {0, 0, 0};
Position Original_Position_Distance_Plan = {0, 0, 0};
Position End_Position_Distance_Plan = {0, 0, 0};
Position Speed_Control = {0, 0, 0};
Position Zero = {0, 0, 0};
float World_x = 0, World_y = 0, World_yaw = 0;
float temp = 0;
float Two_Speed_Abs = 0;
float Speed_Angle = 0;

/**
 * @brief 舵轮底盘解算函数
 *
 * @param[in] none
 *
 * @return none
 *
 * @note 用于运动模型解析
 * Wheel No. 1 is located in front left
 * the rest are arranged counterclockwise
 */
/*
*********************************************************************************************************
*                                            	变量定义
*********************************************************************************************************
*/
//Wheel No. 1 is located in front left
int Nav_i;
float OMG[WHEELNUM + 1] = {0, 0.25 * PI, 0.25 * PI, 0.25 * PI, 0.25 * PI};
float Wheel_Now_Direction[WHEELNUM + 1] = {0};
float Wheel_Last_Direction[WHEELNUM + 1] = {0, 0, 0, 0};
float Wheel_Front_Direction[WHEELNUM + 1] = {0, 0, 0, 0};

float turnFlag[WHEELNUM + 1] = {1, 1, 1, 1,1};
float Vx_part[WHEELNUM + 1] = {0, 0, 0, 0};
float Vy_part[WHEELNUM + 1] = {0, 0, 0, 0};
uint8_t step = 0;
uint8_t step_1 = 0;
int speed_flag = 0;
// int flag_1 = 0;
float Passed_Distance = 0.0;
float omega = 0;
Robot_INFO Robot;
/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/

/**
 * @brief 机器人速度转轮速
 * @param none
 * @return None
 * @note 机器人在自身坐标系下速度转到轮速
 */
void Robot_Velocity_To_Wheel_Speed() // 速度分解
{
  float Vx, Vy, Vw;
  Vx = Robot.Robot_in_self.velocity_exp.Vx;
  Vy = Robot.Robot_in_self.velocity_exp.Vy;
  Vw = -Robot.Robot_in_self.velocity_exp.Vw * Rotate_R; // 车轮到三角中心的距离
  // Motion parameter decomposition
  Vx_part[1] = Vx - Vw * cos(OMG[1]);
  Vy_part[1] = Vy + Vw * sin(OMG[1]);
  Vx_part[2] = Vx - Vw * cos(OMG[2]);
  Vy_part[2] = Vy - Vw * sin(OMG[2]);
  Vx_part[3] = Vx + Vw * cos(OMG[3]);
  Vy_part[3] = Vy - Vw * sin(OMG[3]);
  Vy_part[4] = Vy + Vw * sin(OMG[4]);
  Vx_part[4] = Vx + Vw * cos(OMG[4]);
  for (int i = 1; i <= WHEELNUM; i++)
  {
    Robot.Wheel_Speed_Abs[i] = sqrt((Vx_part[i] * Vx_part[i]) + (Vy_part[i] * Vy_part[i]));
    if (speed_flag == 0 && (Robot.Wheel_Speed_Abs[i] != 0))
    {
      Robot.Wheel_Speed[i] = Robot.Wheel_Speed_Abs[i];
      speed_flag = 1;
    }
    // Robot.Wheel_Speed[i]=turnFlag[i]*Robot.Wheel_Speed_Abs[i];
  }
}
/**
 * @brief 机器人舵向轮角度
 * @param none
 * @return None
 * @note 机器人在自身坐标系下舵向轮旋转角度
 */
float temp_1[5] = {0};
void Robot_Direction_To_Wheel_Direction() // 角度计算
{
  if (!Robot.Wheel_Speed_Abs[1] && !Robot.Wheel_Speed_Abs[2] && !Robot.Wheel_Speed_Abs[3] && !Robot.Wheel_Speed_Abs[4])
  {
//    Robot.Wheel_Speed[1] = 0;
//    Robot.Wheel_Speed[2] = 0;
//    Robot.Wheel_Speed[3] = 0;
//    Robot.Wheel_Speed[4] = 0;
    for(int i=1;i<=4;i++)
        Robot.Wheel_Speed[i] = turnFlag[i] * Robot.Wheel_Speed_Abs[i];

    return;
  }
  for (int i = 1; i <= WHEELNUM; i++)
  {
    Wheel_Now_Direction[i] = atan2(Vy_part[i], Vx_part[i]);
    float change;
    change = (Wheel_Now_Direction[i] - Wheel_Front_Direction[i]);
    if (fabs(change) > 3 * PI / 2)
    {
      if (change < 0)
      {
        temp_1[i] += change + 2 * PI;
      }
      else
      {
        temp_1[i] += change - 2 * PI;
      }
      turnFlag[i] = turnFlag[i];
    }
    else if (fabs(change) > PI / 2)
    {
      if (change > 0)
      {
        temp_1[i] +=change- PI;
      }
      else
      {
        temp_1[i] += change + PI;
      }
      turnFlag[i] = -turnFlag[i];
    }
    else if(change != 0)
    {
      temp_1[i] += change;
      turnFlag[i] = turnFlag[i];
    }
    if(fabs(temp_1[i])>PI)
    {
      if(temp_1[i]>0)
      {
        temp_1[i]-=2*PI;
      }
      else
      {
        temp_1[i]+=2*PI;
      }
    }
    Robot.Wheel_Speed[i] = turnFlag[i] * Robot.Wheel_Speed_Abs[i];
    Robot.Wheel_Direction[i] = temp_1[i];
    Wheel_Front_Direction[i] = Wheel_Now_Direction[i];

    // turnFlag[i] = Robot.Wheel_Speed[i] / fabs(Robot.Wheel_Speed[i]);
    /*Wheel_Now_Direction[i] = atan2(Vy_part[i], Vx_part[i]);
    float change;
    change = Wheel_Now_Direction[i] - Wheel_Front_Direction[i];
    if (fabs(change) > 3 * PI / 2)
    {
      if (change < 0)
      {
        change = (change + 2 * PI);
      }
      else
      {
        change = (change - 2 * PI);
      }
      turnFlag[i] = turnFlag[i];
    }
    else if (fabs(change) > PI / 2)
    {
      if (change > 0)
      {
        change = change - PI;
      }
      else
      {
        change = change + PI;
      }
      turnFlag[i] = -turnFlag[i];
    }
    else
    {
      turnFlag[i] = turnFlag[i];
    }
    Robot.Wheel_Speed[i] = turnFlag[i] * Robot.Wheel_Speed_Abs[i];
    Robot.Wheel_Direction[i] += change;
    Wheel_Front_Direction[i] = Wheel_Now_Direction[i];*/
  }
}
/**
 * @brief 轮速转电机
 * @param none
 * @return None
 *
 */
void Wheel_Speed_To_Motor()
{
  for (int i = 1; i <= WHEELNUM; i++)
  {
    M3508[i].ExpSpeed = (int16_t)(60*19*Robot.Wheel_Speed[i] / (2 * PI * WHEEL_R));
  }
}
/**
 * @brief 转动角度转电机
 * @param none
 * @return None
 *
 */
  int GM6020_unsure[5]={0};
void Wheel_Direction_To_Motor()
{
  for (int i = 1; i <= WHEELNUM; i++)
  {
    
     GM6020_unsure[i] = 8192-(int)(Robot.Wheel_Direction[i] * 8192 / (2*PI)+4096);
    //recify angle
    if(i == 4)
       GM6020_unsure[i] = (GM6020_unsure[i] + 2730)%8192;
    else if(i == 3){
      GM6020_unsure[i] = (GM6020_unsure[i] - 2730);
      if(GM6020_unsure[i]< 0)
         GM6020_unsure[i]+= 8192;//one_ciecle_is_8192
    }
  }
    for (int i = 1; i <= WHEELNUM; i++)//recify_real_x
  {
     GM6020_unsure[i]-= 683;
    if(GM6020_unsure[i] < 0)
        GM6020_unsure[i]+= 8192;
    //recify angle
  }
  for (int i = 1; i <= WHEELNUM; i++)
  {
    GM6020[i].Exparg= GM6020_unsure[i];
  }
}
/**
 * @brief 机器人世界坐标系转自身坐标系速度
 * @param none
 * @return None
 */
void Robot_Velocity_World_To_Self()
{
  float Vx, Vy, Vw;
  Vx = Robot.Robot_in_world.velocity_exp.Vx;
  Vy = Robot.Robot_in_world.velocity_exp.Vy;
  Vw = Robot.Robot_in_world.velocity_exp.Vw;
  Robot.Robot_in_self.velocity_exp.Vx = Vx * cos(Robot.Robot_in_self.position_now.yaw) + Vy * sin(Robot.Robot_in_self.position_now.yaw);
  Robot.Robot_in_self.velocity_exp.Vy = -Vx * sin(Robot.Robot_in_self.position_now.yaw) + Vy * cos(Robot.Robot_in_self.position_now.yaw);
  Robot.Robot_in_self.velocity_exp.Vw = Vw;
}