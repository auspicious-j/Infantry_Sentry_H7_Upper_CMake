#include "USER_RC.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "cmsis_os.h"
#include "UserFreertos.h"
#include "USER_Moto.h"
#include "USER_B2B.h"


uint8_t usart5RxBuf[25]; // 串口5缓冲区
MC6C_RC_t rcInfo_MC6C = {0};//三个遥控器各自的接口
DR16_RC_T rcInfo_DR16 = {0};
ET08_RC_t rcInfo_ET08 = {0};

RC_TypeDef rcInfo = {0};//统一接口

int rc_true_flag;

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;


void RC_Init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5RxBuf, sizeof(usart5RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
}

SwitchState SBUS_SwitchState(int16_t ch)
{
    if (ch > 1500)       return SWITCH_UP;
    else if (ch < 500) return SWITCH_DOWN;
    else                return SWITCH_MID;
}
float DeadZone(float x, float zone)
{
    if (x > -zone && x < zone)
        return 0.0f;
    return x;
}	
void MC6C_ParseSBUS(uint8_t *buf, MC6C_RC_t *rc)
{
    /* 帧头帧尾校验 */
    if (buf[0] != 0x0F || buf[24] != 0x00)
        return;
    /* 16 通道，每通道 11bit */
    rc->ch[0]  = (buf[1]       | buf[2]  << 8) & 0x07FF;
    rc->ch[1]  = (buf[2] >> 3  | buf[3]  << 5) & 0x07FF;
    rc->ch[2]  = (buf[3] >> 6  | buf[4]  << 2 | buf[5]  << 10) & 0x07FF;
    rc->ch[3]  = (buf[5] >> 1  | buf[6]  << 7) & 0x07FF;
    rc->ch[4]  = (buf[6] >> 4  | buf[7]  << 4) & 0x07FF;
    rc->ch[5]  = (buf[7] >> 7  | buf[8]  << 1 | buf[9]  << 9) & 0x07FF;
    rc->ch[6]  = (buf[9] >> 2  | buf[10] << 6) & 0x07FF;
    rc->ch[7]  = (buf[10] >> 5 | buf[11] << 3) & 0x07FF;

    rc->ch[8]  = (buf[12]      | buf[13] << 8) & 0x07FF;
    rc->ch[9]  = (buf[13] >> 3 | buf[14] << 5) & 0x07FF;
    rc->ch[10] = (buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF;
    rc->ch[11] = (buf[16] >> 1 | buf[17] << 7) & 0x07FF;
    rc->ch[12] = (buf[17] >> 4 | buf[18] << 4) & 0x07FF;
    rc->ch[13] = (buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF;
    rc->ch[14] = (buf[20] >> 2 | buf[21] << 6) & 0x07FF;
    rc->ch[15] = (buf[21] >> 5 | buf[22] << 3) & 0x07FF;

    /* flag 字节 */
    uint8_t flag = buf[23];
    rc->lost     = (flag & (1 << 2)) ? 1 : 0;
    rc->failsafe = (flag & (1 << 3)) ? 1 : 0;
		
	rc->left_last = rc->left;
	rc->right_last = rc->right;

	rc->ch1 = rc->ch[0] - 1000;		
	rc->ch2 = rc->ch[1] - 1000;
	rc->ch3 = rc->ch[2] - 1000;
	rc->ch4	= rc->ch[3] - 1000;
	rc->ch1 = DeadZone(rc->ch1,20);
	rc->ch2 = DeadZone(rc->ch2,20);
	rc->ch3 = DeadZone(rc->ch3,20);
	rc->ch4 = DeadZone(rc->ch4,20);
    rc->left  = SBUS_SwitchState(rc->ch[4]);
    rc->right = SBUS_SwitchState(rc->ch[5]);
}

void DR16_ParseSBUS(uint8_t* buff,DR16_RC_T *rc)
{
	rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
	rc->ch1 -= 1024;		//右横
	rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
	rc->ch2 -= 1024;		//右竖
	rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
	rc->ch3 -= 1024;		//左横
	rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
	rc->ch4 -= 1024;		//左竖
	
  /* prevent remote control zero deviation */
	rc->ch1 = DeadZone(rc->ch1,40);
	rc->ch2 = DeadZone(rc->ch2,40); //设静止死区
	rc->ch3 = DeadZone(rc->ch3,40);
	rc->ch4 = DeadZone(rc->ch4,40); //设静止死区
	
	rc->left_last = rc->left;
	rc->right_last = rc->right;

	rc->left = ((buff[5] >> 4) & 0x000C) >> 2;  //sw1   中间是3，上边是1，下边是2
	rc->right = (buff[5] >> 4) & 0x0003;        //sw2

	rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10] | (buff[11] << 8);

	rc->mouse.l = buff[12];
	rc->mouse.r = buff[13];

	rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
	rc->wheel = (buff[16] | buff[17] << 8) - 1024;
}


void ET08_ParseSBUS(uint8_t *buff, ET08_RC_t *rc)
{
    /* 帧头帧尾校验 */
    if (buff[0] != 0x0F || buff[24] != 0x00)
        return;
    /* 16 通道，每通道 11bit */
    rc->ch[0]  = (buff[1]       | buff[2]  << 8) & 0x07FF;
    rc->ch[1]  = (buff[2] >> 3  | buff[3]  << 5) & 0x07FF;
    rc->ch[2]  = (buff[3] >> 6  | buff[4]  << 2 | buff[5]  << 10) & 0x07FF;
    rc->ch[3]  = (buff[5] >> 1  | buff[6]  << 7) & 0x07FF;
    rc->ch[4]  = (buff[6] >> 4  | buff[7]  << 4) & 0x07FF;
    rc->ch[5]  = (buff[7] >> 7  | buff[8]  << 1 | buff[9]  << 9) & 0x07FF;
    rc->ch[6]  = (buff[9] >> 2  | buff[10] << 6) & 0x07FF;
    rc->ch[7]  = (buff[10] >> 5 | buff[11] << 3) & 0x07FF;

    rc->ch[8]  = (buff[12]      | buff[13] << 8) & 0x07FF;
    rc->ch[9]  = (buff[13] >> 3 | buff[14] << 5) & 0x07FF;
    rc->ch[10] = (buff[14] >> 6 | buff[15] << 2 | buff[16] << 10) & 0x07FF;
    rc->ch[11] = (buff[16] >> 1 | buff[17] << 7) & 0x07FF;
    rc->ch[12] = (buff[17] >> 4 | buff[18] << 4) & 0x07FF;
    rc->ch[13] = (buff[18] >> 7 | buff[19] << 1 | buff[20] << 9) & 0x07FF;
    rc->ch[14] = (buff[20] >> 2 | buff[21] << 6) & 0x07FF;
    rc->ch[15] = (buff[21] >> 5 | buff[22] << 3) & 0x07FF;

    uint8_t flag = buff[23];
    rc->lost     = (flag & (1 << 2)) ? 1 : 0;
    rc->failsafe = (flag & (1 << 3)) ? 1 : 0;

	rc->ch1 = rc->ch[0] - 1024;//右横
	rc->ch2 = rc->ch[1] - 1024;//右竖
	rc->ch3 = rc->ch[2] - 1024;//左横
	rc->ch4 = rc->ch[3] - 1024;//左竖	
  /* prevent remote control zero deviation */
  	rc->ch1 = DeadZone(rc->ch1,10);
	rc->ch2 = DeadZone(rc->ch2,10); 
	rc->ch3 = DeadZone(rc->ch3,10);
	rc->ch4 = DeadZone(rc->ch4,10); //设静止死区

	rc->SA_last = rc->SA;
	rc->SB_last = rc->SB;
	rc->SC_last = rc->SC;
	rc->SD_last = rc->SD;

    rc->SA = SBUS_SwitchState(rc->ch[4]);
    rc->SB = SBUS_SwitchState(rc->ch[5]);
    rc->SC = SBUS_SwitchState(rc->ch[6]);
    rc->SD = SBUS_SwitchState(rc->ch[7]);

}

//统一接口
void MC6C_ToUnified(MC6C_RC_t *rc_MC6C, RC_TypeDef *rc)
{
	rc->ch1 = rc_MC6C->ch1;		
	rc->ch2 = rc_MC6C->ch2;
	rc->ch3 = rc_MC6C->ch3;
	rc->ch4	= rc_MC6C->ch4;

	rc->left = rc_MC6C->left;
	rc->right = rc_MC6C->right;
	rc->left_last = rc_MC6C->left_last;
	rc->right_last = rc_MC6C->right_last;
	rc->wheel = 0;
	rc->failsafe = rc_MC6C->failsafe;
	rc->lost = rc_MC6C->lost;
}

void DR16_ToUnified(DR16_RC_T *rc_DR16, RC_TypeDef *rc)
{
	rc->ch1 = rc_DR16->ch1;		
	rc->ch2 = rc_DR16->ch2;
	rc->ch3 = rc_DR16->ch3;
	rc->ch4	= -rc_DR16->ch4;

	rc->left = rc_DR16->left;
	rc->right = rc_DR16->right;
	rc->left_last = rc_DR16->left_last;
	rc->right_last = rc_DR16->right_last;

	rc->mouse.x = rc_DR16->mouse.x;
	rc->mouse.y = rc_DR16->mouse.y;
	rc->mouse.z = rc_DR16->mouse.z;
	rc->mouse.l = rc_DR16->mouse.l;
	rc->mouse.r = rc_DR16->mouse.r;
	rc->kb.key_code = rc_DR16->kb.key_code;

	rc->wheel = rc_DR16->wheel;
}

void ET08_ToUnified(ET08_RC_t *rc_ET08, RC_TypeDef *rc)
{
	rc->ch1 = rc_ET08->ch1;		
	rc->ch2 = rc_ET08->ch2;
	rc->ch3 = rc_ET08->ch3;
	rc->ch4	= rc_ET08->ch4;

	rc->lleft = rc_ET08->SA;
	rc->left = rc_ET08->SB;
	rc->right = rc_ET08->SC;
	rc->rright = rc_ET08->SD;

	rc->lleft_last = rc_ET08->SA_last;
	rc->left_last = rc_ET08->SB_last;
	rc->right_last = rc_ET08->SC_last;
	rc->rright_last = rc_ET08->SD_last;

	rc->wheel = 0;
	rc->failsafe = rc_ET08->failsafe;
	rc->lost = rc_ET08->lost;
}

// 串口5空闲中断回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart2)
	{
		B2B_Receive();
	}
	if (huart == &huart5)
	{
#if (USER_RC_TYPE == USER_RC_TYPE_MC6C)
		MC6C_ParseSBUS(usart5RxBuf, &rcInfo_MC6C);
		MC6C_ToUnified(&rcInfo_MC6C, &rcInfo);
#elif (USER_RC_TYPE == USER_RC_TYPE_DR16)
		DR16_ParseSBUS(usart5RxBuf,&rcInfo_DR16);
		DR16_ToUnified(&rcInfo_DR16, &rcInfo);
#else
		ET08_ParseSBUS(usart5RxBuf, &rcInfo_ET08);
		ET08_ToUnified(&rcInfo_ET08, &rcInfo);
#endif
		rc_true_flag = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5RxBuf, sizeof(usart5RxBuf));
		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	}
}

void Task_RC_Callback()
{
	/**********特殊情况处理*********************/
	if (rcInfo.right == 2) // 遥控器右拨码开关向下，急停
	{

    	disable_motor_mode(&hfdcan2,0x01,MIT_MODE);
		HAL_Delay(10);
		USER_CAN_SetMotorCurrent(&hfdcan1, 0x1FF, 0, 0, 0, 0);
		HAL_Delay(10);
		USER_CAN_SetMotorCurrent(&hfdcan1, 0x200, 0, 0, 0, 0); // 关断电机
		HAL_Delay(10);
		USER_CAN_SetMotorCurrent(&hfdcan2,0x1FF,0,0,0,0);//关断电机
		STOPFLAG = 1;
    	B2B_Transmit();
		osThreadResume(ErrorTaskHandle); // 恢复错误任务 饿死其他任务
	}
}

/************************freertos任务****************************/
void OS_RcCallback(void const *argument)
{

	for (;;)
	{
		rc_true_flag++;
		if (rc_true_flag >= 100)//长时间未接收到遥控器 手动重新接收
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5RxBuf, sizeof(usart5RxBuf));
			__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
		}
		Task_RC_Callback();
		osDelay(15);
	}
}
