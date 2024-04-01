#include "bsp_can.h"

CANx_t CAN_1,CAN_2;
char Selection=0;

uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//电机使能命令
uint8_t Data_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//电机失能命令
uint8_t Data_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};	//电机保存零点命令


/**
 * @brief  这里对ID为0x02、0x03、0x04的3个电机进行依次使能，在对电机进行控制前进行使能
 * @param  void     	
 * @param  vodi      
 * @param  void    	
 * @param  void      	
 */
void Motor_enable(void)
{
	#if Motar_mode==0
	CANx_SendStdData(&hcan1,0x02,Data_Enable,8);	
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x03,Data_Enable,8);
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x04,Data_Enable,8);
	HAL_Delay(10);
	#elif Motar_mode==1
	CANx_SendStdData(&hcan1,0x102,Data_Enable,8);	
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x103,Data_Enable,8);
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x104,Data_Enable,8);
	HAL_Delay(10);
	#elif Motar_mode==2
	CANx_SendStdData(&hcan1,0x202,Data_Enable,8);	
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x203,Data_Enable,8);
	HAL_Delay(10);
	CANx_SendStdData(&hcan1,0x204,Data_Enable,8);
	HAL_Delay(10);
	#endif

}

/**
* @brief  这里对ID为0x02、0x03、0x04的3个电机进行依次控制，在freertos.c中CAN_Send_Task任务中1ms执行一次，
这里使用了3个电机也就是每个电机3ms发送一次,注意多个电机不能同时一起发，发送频率过快，则会使有些ID没发送到
 * @param  void     	
 * @param  vodi      
 * @param  void    	
 * @param  void      	
 */

void Motor_control(void)
{
	#if Motar_mode==0
		switch(Selection)
		{
			case 0:
			{
				MIT_CtrlMotor(&hcan1,0X02, 0, 2,0, 1, 0);	  //IMT控制模式对ID为0X02电机进行速度控制发送
				Selection++;
				break;
			}
			case 1:
			{
				MIT_CtrlMotor(&hcan1,0X03, 0, 2,0, 1, 0);			//IMT控制模式对ID为0X03电机进行速度控制发送
				Selection++;
				break;
			}
			case 2:
			{
				MIT_CtrlMotor(&hcan1,0X04, 0, 2,0, 1, 0);			//IMT控制模式对ID为0X04电机进行速度控制发送
				Selection=0;
				break;
			}
		}
	#elif Motar_mode==1
		switch(Selection)
		{
			case 0:
			{
				PosSpeed_CtrlMotor(&hcan1,0X102, 10, 5);	  //位置速度控制模式对ID为0X02电机进行速度控制发送
				Selection++;
				break;
			}
			case 1:
			{
				PosSpeed_CtrlMotor(&hcan1,0X103,  10, 5);			//位置速度控制模式对ID为0X03电机进行速度控制发送
				Selection++;
				break;
			}
			case 2:
			{
				PosSpeed_CtrlMotor(&hcan1,0X104,  10, 5);			//位置速度控制模式对ID为0X04电机进行速度控制发送
				Selection=0;
				break;
			}
		}
	#elif Motar_mode==2
		switch(Selection)
		{
			case 0:
			{
				Speed_CtrlMotor(&hcan1,0X202, 5);	  //速度控制模式对ID为0X02电机进行速度控制发送
				Selection++;
				break;
			}
			case 1:
			{
				Speed_CtrlMotor(&hcan1,0X203, 5);			//速度控制模式对ID为0X03电机进行速度控制发送
				Selection++;
				break;
			}
			case 2:
			{
				Speed_CtrlMotor(&hcan1,0X204, 5);			//速度控制模式对ID为0X04电机进行速度控制发送
				Selection=0;
				break;
			}
		}
	#endif


}

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

int float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits///
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief  MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定值
 */
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,
float _KP, float _KD, float _torq)
 { 
	static CAN_TxHeaderTypeDef   Tx_Header;
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	Tx_Header.StdId=id;
	Tx_Header.IDE=CAN_ID_STD;
	Tx_Header.RTR=CAN_RTR_DATA;
	Tx_Header.DLC=0x08;
	
	CAN_1.Tx_Data[0] = (pos_tmp >> 8);
	CAN_1.Tx_Data[1] = pos_tmp;
	CAN_1.Tx_Data[2] = (vel_tmp >> 4);
	CAN_1.Tx_Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	CAN_1.Tx_Data[4] = kp_tmp;
	CAN_1.Tx_Data[5] = (kd_tmp >> 4);
	CAN_1.Tx_Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	CAN_1.Tx_Data[7] = tor_tmp;
	 
	 //寻空邮箱发送数据
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
        }
   }
 }

/**
 * @brief  位置速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 */
void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel)
{
    static CAN_TxHeaderTypeDef Tx_Header;
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

		Tx_Header.StdId=id;
		Tx_Header.IDE=CAN_ID_STD;
		Tx_Header.RTR=CAN_RTR_DATA;
		Tx_Header.DLC=0x08;

    CAN_1.Tx_Data[0] = *pbuf;;
    CAN_1.Tx_Data[1] = *(pbuf+1);
    CAN_1.Tx_Data[2] = *(pbuf+2);
    CAN_1.Tx_Data[3] = *(pbuf+3);
    CAN_1.Tx_Data[4] = *vbuf;
    CAN_1.Tx_Data[5] = *(vbuf+1);
    CAN_1.Tx_Data[6] = *(vbuf+2);
    CAN_1.Tx_Data[7] = *(vbuf+3);

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}
 
/**
 * @brief  速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _vel   速度给定
 */
void Speed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _vel)
{
		static CAN_TxHeaderTypeDef   Tx_Header;
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;

    Tx_Header.StdId = ID;
    Tx_Header.IDE = CAN_ID_STD;
    Tx_Header.RTR = CAN_RTR_DATA;
    Tx_Header.DLC = 0x04;

    CAN_1.Tx_Data[0] = *vbuf;
    CAN_1.Tx_Data[1] = *(vbuf+1);
    CAN_1.Tx_Data[2] = *(vbuf+2);
    CAN_1.Tx_Data[3] = *(vbuf+3);

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, CAN_1.Tx_Data, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}
 



/*
 * @brief: CAN Receive Message.
 * @param: "RxData[]" will store the message which has been received, which length must between 0 and 8.
 * @retval: receive status.
 */
 /*CAN中断接受*/
 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	if(hcan->Instance==CAN1)
	{
		
	if(HAL_CAN_GetRxMessage(&hcan1,CAN_FILTER_FIFO0,&CAN_1.Rx_pHeader,CAN_1.RxData)==HAL_OK)//获取数据
	{
	if(CAN_1.Rx_pHeader.StdId == 0X02)
 {

	CAN_1.p_int[0]=(CAN_1.RxData[1]<<8)|CAN_1.RxData[2];
	CAN_1.v_int[0]=(CAN_1.RxData[3]<<4)|(CAN_1.RxData[4]>>4);
	CAN_1.t_int[0]=((CAN_1.RxData[4]&0xF)<<8)|CAN_1.RxData[5];
	CAN_1.position[0] = uint_to_float(CAN_1.p_int[0], P_MIN, P_MAX, 16); // (-12.5,12.5)
	CAN_1.velocity[0] = uint_to_float(CAN_1.v_int[0], V_MIN, V_MAX, 12); // (-45.0,45.0)
	CAN_1.torque[0] = uint_to_float(CAN_1.t_int[0], T_MIN, T_MAX, 12); // (-18.0,18.0)
 }
 if(CAN_1.Rx_pHeader.StdId == 0X03)
 {
	CAN_1.p_int[1]=(CAN_1.RxData[1]<<8)|CAN_1.RxData[2];
	CAN_1.v_int[1]=(CAN_1.RxData[3]<<4)|(CAN_1.RxData[4]>>4);
	CAN_1.t_int[1]=((CAN_1.RxData[4]&0xF)<<8)|CAN_1.RxData[5];
	CAN_1.position[1] = uint_to_float(CAN_1.p_int[1], P_MIN, P_MAX, 16); // (-12.5,12.5)
	CAN_1.velocity[1] = uint_to_float(CAN_1.v_int[1], V_MIN, V_MAX, 12); // (-45.0,45.0)
	CAN_1.torque[1] = uint_to_float(CAN_1.t_int[1], T_MIN, T_MAX, 12); // (-18.0,18.0)
 }
  if(CAN_1.Rx_pHeader.StdId == 0X04)
 {
	CAN_1.p_int[2]=(CAN_1.RxData[1]<<8)|CAN_1.RxData[2];
	CAN_1.v_int[2]=(CAN_1.RxData[3]<<4)|(CAN_1.RxData[4]>>4);
	CAN_1.t_int[2]=((CAN_1.RxData[4]&0xF)<<8)|CAN_1.RxData[5];
	CAN_1.position[2] = uint_to_float(CAN_1.p_int[2], P_MIN, P_MAX, 16); // (-12.5,12.5)
	CAN_1.velocity[2] = uint_to_float(CAN_1.v_int[2], V_MIN, V_MAX, 12); // (-45.0,45.0)
	CAN_1.torque[2] = uint_to_float(CAN_1.t_int[2], T_MIN, T_MAX, 12); // (-18.0,18.0)
 }
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	//激活CAN中断通知
	}
}
 
}


