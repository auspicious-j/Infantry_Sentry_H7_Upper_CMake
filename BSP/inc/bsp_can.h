#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include "fdcan.h"
#include "cmsis_os.h"

//can????? debug?
typedef struct
{
	HAL_StatusTypeDef can1_user_init_error,can2_user_init_error,can3_user_init_error;
	uint16_t can1_send_error,can2_send_error,can3_send_error;
	uint16_t can1_receive_error,can2_receive_error,can3_receive_error;
}CanState;

//can初始化
void CAN_Init(void);

/****??????****/
void USER_CAN_Send(FDCAN_HandleTypeDef* hfdcan,int16_t StdId,uint8_t* tx_data);

#endif
