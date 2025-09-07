#ifndef __bsp_can_fd_H
#define __bsp_can_fd_H

#include "stm32h7xx_hal.h"


typedef struct 
{
  uint8_t super_cap_power;
	uint8_t cap_state;
	
	
	
}can_tx_data_t;



typedef struct 
{
	uint8_t charge_sign1;
	uint8_t data[8];
	uint8_t gyro_sign;
	uint8_t super_cap_sign;
	uint8_t frictiongear_sign;
	uint8_t vision_sign;
	uint8_t power_buff;
	uint8_t power_limit;
	uint8_t robot_id;
	uint8_t magazine_sign;
	can_tx_data_t can_tx_data;
	uint8_t Violent_Model;
	int16_t rc_ch1;
	uint8_t Model;
	uint8_t Infantry_Down;
	int cnt;
	float angle_gap;
	uint8_t charge_power;
	int16_t page;
	
}robot_data;


extern robot_data can_data;


 void Can_Send_Data_Can(void);
void receive(robot_data* can , FDCAN_HandleTypeDef* hcan);
void FDCAN_Init(void);




#endif

