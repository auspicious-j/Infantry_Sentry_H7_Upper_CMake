#include "USER_B2B.h"
#include "Chassis.h"
#include "usart.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "gimbal.h"
#include <string.h>
#include "vision.h"
#include "Judge.h"

extern DMA_HandleTypeDef hdma_usart2_rx;

/* 需要用到的接收变量*/
uint8_t usart2RxBuf[256]; // 串口2缓冲区
uint8_t STOPFLAG = 0;   //是1下板急停
uint8_t FEEDBACK = 0;		//是0下板急停

uint32_t receive_times;

/* 需要用到的发送变量*/
uint8_t usart2TxBuf[64];

// 板间通信初始化
void B2B_Init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2RxBuf, sizeof(usart2RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void B2B_Transmit()
{
		usart2TxBuf[0] = 0xAA;				 // 帧头
		for (uint8_t i = 0; i < 4; i++)
		{
			usart2TxBuf[1 + i * 2] = chassis.motors[i].targetSpeed;
			usart2TxBuf[1 + i * 2 + 1] = chassis.motors[i].targetSpeed >> 8;
		} // 1-9 轮电机目标速度

		usart2TxBuf[62]  = STOPFLAG;    // 急停标志
		usart2TxBuf[63] = 0xFE;				 // 帧尾

		HAL_UART_Transmit_DMA(&huart2, usart2TxBuf, sizeof(usart2TxBuf));
}


void B2B_Receive(void)
{
	if (usart2RxBuf[0] == 0xAB && usart2RxBuf[63] == 0xFD)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			chassis.motors[i].speed = (int16_t)usart2RxBuf[1 + i * 2] | (int16_t)usart2RxBuf[1 + i * 2 + 1] << 8;
		}//轮电机当前速度	1-8
		SET_WHEELSPEED_MAX = (uint16_t)usart2RxBuf[9] | (uint16_t)usart2RxBuf[10] << 8; // 轮电机速度上限	9-10
		{
			float v;
			uint8_t *p = (uint8_t *)&v;
			p[0] = usart2RxBuf[11];
			p[1] = usart2RxBuf[12];
			p[2] = usart2RxBuf[13];
			p[3] = usart2RxBuf[14];
			chassis.move.maxPower = v;
		}	//底盘吃缓冲能量后最大功率
		
		memcpy(&USER_JudgeData, &usart2RxBuf[15], sizeof(JudgeData_t));
		//裁判系统数据 15-43
		FEEDBACK = usart2RxBuf[62];
	}
}

/************************freertos任务****************************/
void OS_Board2BoardCallback(void const *argument)
{
	while (1)
	{
		B2B_Transmit();
		osDelay(1);
	}
}
