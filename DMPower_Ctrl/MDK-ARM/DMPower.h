#ifndef __DMPOWER__
#define __DMPOWER__

#include "DMPower.h"
#include "stm32f4xx_hal.h"
#define MOTOR1  0x01   //1电机ID
#define MOTOR2  0x02   //2电机ID
#define MOTOR3  0x03   //3电机ID
#define P_MIN   -12.5  //位置最小值
#define P_MAX   12.5   //位置最大值
#define V_MIN   -45    //速度最小值
#define V_MAX   45     //速度最大值
#define KP_MIN  0      //Kp最小值
#define KP_MAX  500    //Kp最大值
#define KD_MIN  0      //Kd最小值
#define KD_MAX  5      //Kd最大值
#define T_MIN   -18    //转矩最大值
#define T_MAX   18     //转矩最小值

//电机结构体
typedef struct
{
  int p_int;
  int v_int;
  int t_int;
  int position;
  int velocity;
  int torque;
}Motor_t;
//发送包
typedef struct
{
	CAN_TxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_TxPacketTypeDef;
//接收包
typedef struct
{
	CAN_RxHeaderTypeDef hdr;
	uint8_t payload[8];
}CAN_RxPacketTypeDef;

#endif /* __DMPOWER__ */