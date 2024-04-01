#ifndef __BSP_CAN_H
#define __BSP_CAN_H
#include "can.h"


#define Motar_mode 0	//设置模式为何种模式，为0为IMT模式，为1为位置速度模式，为2为速度模式

#define P_MIN -12.5		//位置最小值
#define P_MAX 12.5		//位置最大值
#define V_MIN -45			//速度最小值
#define V_MAX 45			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -18			//转矩最大值
#define T_MAX 18			//转矩最小值

typedef struct
{
	int p_int[3],v_int[3],t_int[3];						//这里可根据电机数目自行修改，读取三个电机的位置、速度、转矩
	float position[3],velocity[3],torque[3];	//三个电机的位置、速度、转矩解析存储
	uint8_t  Tx_Data[8];												//数据发送存储
	uint8_t  RxData[8];												//数据接收存储
	CAN_RxHeaderTypeDef Rx_pHeader;
}CANx_t;
extern CANx_t CAN_1,CAN_2;


void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,
float _KP, float _KD, float _torq);
void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel);
void Speed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _vel);
void Motor_enable(void);
void Motor_control(void);

#endif
