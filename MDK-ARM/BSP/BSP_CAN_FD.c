/***                           BSP_CAN_FD.c
 *
 *                 .-~~~~~~~~~-._       _.-~~~~~~~~~-.
 *             __.'              ~.   .~              `.__
 *           .'//                  \./                  \\`.
 *         .'//                     |                     \\`.
 *       .'// .-~"""""""~~~~-._     |     _,-~~~~"""""""~-. \\`.
 *     .'//.-"                 `-.  |  .-'                 "-.\\`.
 *   .'//______.============-..   \ | /   ..-============.______\\`.
 * .'______________________________\|/______________________________`.   /畅游知识的海洋,学习使我快乐/
 *
 */



#include "BSP_CAN_FD.h"
#include "PWM_Control.h"
#include "fdcan.h"
#include "lib.h"
int a;


/**********************************************************************************
  * @brief                                       can初始化
  * @author                                      大工
  * @param                                       含过滤器配置
  * @retval                                      返回无
  *********************************************************************************/
	void FDCAN_Init()
{
	  //过滤器配置
		FDCAN_FilterTypeDef sFilterConfig;
		sFilterConfig.IdType = FDCAN_STANDARD_ID;
		sFilterConfig.FilterIndex = 0;
		sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
		sFilterConfig.FilterID1 = 0x101;
		sFilterConfig.FilterID2 = 0x100;
		if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
		{
			Error_Handler();
		}
		//开启fdcan1
		if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
		{
			Error_Handler();
		}
		if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
		{
			Error_Handler();
		}
		if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
		{
			Error_Handler();
		}
		if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0) != HAL_OK) 
		{
		  Error_Handler(); 
		}
				
}



/**********************can数据发送******************/
 void Can_Send_Data_Can()//发送函数
{
	
	uint8_t data[8];
	data[0]=(uint8_t)hrpwm.Super_Cap_Power;
if(adc.Power>-128&&adc.Power<128)
	  data[1]=(uint8_t)(adc.Power+128);
	else if(adc.Power>=128)
		data[1]=255;
	else if(adc.Power<=-128)
		data[1]=0;
	TxHeader1.Identifier = 0x301;
	TxHeader1.IdType = FDCAN_STANDARD_ID;
	TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader1.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader1.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader1.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader1.TxEventFifoControl =FDCAN_STORE_TX_EVENTS ;//
	TxHeader1.MessageMarker = 0;	//marker++;
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader1,data) != HAL_OK)//或者换while失败后初始化
	{ }//发送数据函数
	TxHeader1.TxEventFifoControl =FDCAN_NO_TX_EVENTS ;//FDCAN_STORE_TX_EVENTS存储信息，以便于调试can是否发出去了FDCAN_NO_TX_EVENTS不存储信息，节省资源
}






/**********************************************************************************
  * @brief                                       can接收中断，发送1ms轮训发送
  * @author                                      大工
  * @param                                       中断回调函数
  * @retval                                      返回无
  *********************************************************************************/
uint8_t can_rx_data[8];
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    
//  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
//  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1,can_rx_data) != HAL_OK)
    {//接收函数
    Error_Handler();
    }

    /* Display LEDx */
    if (RxHeader1.Identifier == 0x100)
    {
			receive(&can_data,&hfdcan1);//将接收到的can数据赋值
			can_data.cnt++;
    }
   
}


/**********************************************************************************
  * @brief                                       can数据处理
  * @author                                      ？？？
  * @param                                       处理接收到的数据
  * @retval                                      返回无
  *********************************************************************************/
robot_data can_data;
void receive(robot_data* can , FDCAN_HandleTypeDef* hcan)
{  
		can->data[0]= can_rx_data[0];
		can->data[1]= can_rx_data[1];
		can->data[2]= can_rx_data[2];
		can->data[3]= can_rx_data[3];
	  can->data[4]= can_rx_data[4];
	  
	  //下供步兵专用
//	  can->super_cap_sign=(uint8_t)(can->data[0])&3;//0,1,2,不加速，一级加速和二级加速
//	  can->frictiongear_sign=(uint8_t)(can->data[0]>>3)&1;//摩擦轮开关标志
//		can->gyro_sign=(uint8_t)(can->data[0]>>4)&1;//小陀螺标志
//	  can->vision_sign=(uint8_t)(can->data[0]>>5)&1;//自瞄开标志
//		can->magazine_sign=(uint8_t)(can->data[0]>>6)&3;//拨弹盘开关标志
//		can->robot_id=(uint8_t)can->data[3];//红蓝方id，此位用于判断是否开启UI任务和舵机任务
//		can->angle_gap=(uint8_t)(((can->data[0]>>2)&63)-25)*3.14159f/25;//底盘位置线
    
	  //通用
	can->charge_sign1=(uint8_t)(can->data[0]);//1充2放
	  can->power_limit=(uint8_t)(can->data[1]);//限制功率
	  can->power_buff=(uint8_t)(can->data[2]);//缓冲能量
	  if(can->power_limit>Max_Power_Limit)  can->power_limit=Max_Power_Limit;
			can->charge_power	=(uint8_t)(can->data[3]);
		//模式,后两位
		can->Model=(uint8_t)(can->data[4])&7;//
																				 //1充放电全自动
		                                     //2只充电不放电
																				 //3不充电也不放电
																				 //离线模式默认充放电全自动，1模式
																				 
	
}



