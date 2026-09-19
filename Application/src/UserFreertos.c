#include "UserFreertos.h"
#include "USER_Moto.h"
#include "USER_B2B.h"
// #include "cmsis_armcc.h"
#include "USER_RC.h"
#include "fdcan.h"
#include "usart.h"
#include "bsp_ws2812.h"
#include <stdint.h>

extern DMA_HandleTypeDef hdma_usart2_rx;
extern uint8_t usart2RxBuf[256];



		
void OS_ErrorCallback(void const * argument)
{
	//	B2B_Init();
	osThreadSuspend(ErrorTaskHandle); //第一次执行挂起自身 
	CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
	disable_motor_mode(&hfdcan2,0x01,MIT_MODE);
	disable_motor_mode(&hfdcan2,0x02,MIT_MODE);
	disable_motor_mode(&hfdcan2,0x03,MIT_MODE);

	USER_CAN_SetMotorCurrent(&hfdcan2,0x1FF,0,0,0,0);//关断电机
	for(;;)
	{
	//B2B_Init();
		USER_CAN_SetMotorCurrent(&hfdcan2,0x1FF,0,0,0,0);//关断电机
		disable_motor_mode(&hfdcan2,0x01,MIT_MODE);
		disable_motor_mode(&hfdcan2,0x02,MIT_MODE);
		disable_motor_mode(&hfdcan2,0x03,MIT_MODE);
		HAL_Delay(1);
		if (FEEDBACK !=1)  //返回值不对  向下板发送停止信息
		{
			STOPFLAG = 1;						 
			B2B_Transmit();
		}
		if (rcInfo.right != 2 && rcInfo.right != 0)
			HAL_NVIC_SystemReset(); // 右拨杆回到中间重启系统
		//红灯闪烁    
		WS2812_Set(0, 50, 0, 0);
		HAL_Delay(75);
		WS2812_Set(0, 0, 0, 0);
		HAL_Delay(75);
	}//无os_delay 最高优先级 
}


