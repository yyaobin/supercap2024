/***                           BSP_CAN_FD.c
 *
 *                 .-~~~~~~~~~-._       _.-~~~~~~~~~-.
 *             __.'              ~.   .~              `.__
 *           .'//                  \./                  \\`.
 *         .'//                     |                     \\`.
 *       .'// .-~"""""""~~~~-._     |     _,-~~~~"""""""~-. \\`.
 *     .'//.-"                 `-.  |  .-'                 "-.\\`.
 *   .'//______.============-..   \ | /   ..-============.______\\`.
 * .'______________________________\|/______________________________`.   /����֪ʶ�ĺ���,ѧϰʹ�ҿ���/
 *
 */



#include "BSP_CAN_FD.h"
#include "PWM_Control.h"
#include "fdcan.h"
#include "lib.h"
int a;


/**********************************************************************************
  * @brief                                       can��ʼ��
  * @author                                      ��
  * @param                                       ������������
  * @retval                                      ������
  *********************************************************************************/
	void FDCAN_Init()
{
	  //����������
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
		//����fdcan1
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



/**********************can���ݷ���******************/
 void Can_Send_Data_Can()//���ͺ���
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
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader1,data) != HAL_OK)//���߻�whileʧ�ܺ��ʼ��
	{ }//�������ݺ���
	TxHeader1.TxEventFifoControl =FDCAN_NO_TX_EVENTS ;//FDCAN_STORE_TX_EVENTS�洢��Ϣ���Ա��ڵ���can�Ƿ񷢳�ȥ��FDCAN_NO_TX_EVENTS���洢��Ϣ����ʡ��Դ
}






/**********************************************************************************
  * @brief                                       can�����жϣ�����1ms��ѵ����
  * @author                                      ��
  * @param                                       �жϻص�����
  * @retval                                      ������
  *********************************************************************************/
uint8_t can_rx_data[8];
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    
//  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
//  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1,can_rx_data) != HAL_OK)
    {//���պ���
    Error_Handler();
    }

    /* Display LEDx */
    if (RxHeader1.Identifier == 0x100)
    {
			receive(&can_data,&hfdcan1);//�����յ���can���ݸ�ֵ
			can_data.cnt++;
    }
   
}


/**********************************************************************************
  * @brief                                       can���ݴ���
  * @author                                      ������
  * @param                                       ������յ�������
  * @retval                                      ������
  *********************************************************************************/
robot_data can_data;
void receive(robot_data* can , FDCAN_HandleTypeDef* hcan)
{  
		can->data[0]= can_rx_data[0];
		can->data[1]= can_rx_data[1];
		can->data[2]= can_rx_data[2];
		can->data[3]= can_rx_data[3];
	  can->data[4]= can_rx_data[4];
	  
	  //�¹�����ר��
//	  can->super_cap_sign=(uint8_t)(can->data[0])&3;//0,1,2,�����٣�һ�����ٺͶ�������
//	  can->frictiongear_sign=(uint8_t)(can->data[0]>>3)&1;//Ħ���ֿ��ر�־
//		can->gyro_sign=(uint8_t)(can->data[0]>>4)&1;//С���ݱ�־
//	  can->vision_sign=(uint8_t)(can->data[0]>>5)&1;//���鿪��־
//		can->magazine_sign=(uint8_t)(can->data[0]>>6)&3;//�����̿��ر�־
//		can->robot_id=(uint8_t)can->data[3];//������id����λ�����ж��Ƿ���UI����Ͷ������
//		can->angle_gap=(uint8_t)(((can->data[0]>>2)&63)-25)*3.14159f/25;//����λ����
    
	  //ͨ��
	can->charge_sign1=(uint8_t)(can->data[0]);//1��2��
	  can->power_limit=(uint8_t)(can->data[1]);//���ƹ���
	  can->power_buff=(uint8_t)(can->data[2]);//��������
	  if(can->power_limit>Max_Power_Limit)  can->power_limit=Max_Power_Limit;
			can->charge_power	=(uint8_t)(can->data[3]);
		//ģʽ,����λ
		can->Model=(uint8_t)(can->data[4])&7;//
																				 //1��ŵ�ȫ�Զ�
		                                     //2ֻ��粻�ŵ�
																				 //3�����Ҳ���ŵ�
																				 //����ģʽĬ�ϳ�ŵ�ȫ�Զ���1ģʽ
																				 
	
}



