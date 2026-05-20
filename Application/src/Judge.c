#include "Judge.h"
#include "string.h"
#include "cRc.h"
#include "usart.h"
#include "userfreertos.h"
#include "usart.h"
#include "USER_B2B.h"
#include "myQueue.h"
#include <stdint.h>
#define DISABLE_JUDGE_TASK

/*****************系统数据定义**********************/
ext_game_status_t GameState;						   // 0x0001
ext_game_result_t GameResult;						   // 0x0002
ext_game_robot_HP_t GameRobotHP;					   // 0x0003
ext_event_data_t EventData;							   // 0x0101
ext_referee_warning_t RefereeWarning;				   // 0x0104
ext_dart_info_t DartRemainingTime;					   // 0x0105	
ext_game_robot_status_t GameRobotState;				   // 0x0201
ext_power_heat_data_t PowerHeatData;				   // 0x0202
ext_game_robot_pos_t GameRobotPos;					   // 0x0203
ext_buff_musk_t BuffMusk;							   // 0x0204
ext_robot_hurt_t RobotHurt;							   // 0x0206
ext_shoot_data_t ShootData;							   // 0x0207
ext_bullet_remaining_t BulletRemaining;				   // 0x0208
ext_rfid_status_t RfidStatus;						   // 0x0209
ext_dart_client_cmd_t DartClientCmd;				   // 0x020A
ext_sentry_info_t SentryDecision;					   // 0x020D


xFrameHeader FrameHeader; // 发送帧头信息
/****************************************************/

JudgeData_t USER_JudgeData;
USER_SentryCmd_t USER_SentryCmd; //哨兵机器人指令数据
ext_CommunatianData_t USER_CommunatianData; //机器人交互信息数据帧
ext_sentry_cmd_t SentryCmd; //哨兵机器人指令数据帧 发给裁判系统

bool Judge_Data_TF = FALSE; // 裁判数据是否可用,辅助函数调用

// 发送队列
Queue judgeQueue = EMPTY_QUEUE;
// 发送队列数据保存区
JudgeTxFrame judgeQueueBuf[JUDGE_QUEUE_SIZE];

// 串口接收缓冲区
uint8_t usart1RxBuf[JUDGE_MAX_RX_LENGTH];

uint16_t shootNum = 0; // 统计发弹量
int transmitNum = 0; // 统计发送次数

/**************裁判系统数据辅助****************/

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  缓存数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
bool JUDGE_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE; // 数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误

	uint16_t judge_length; // 统计一帧数据长度
	int CmdID = 0;		   // 数据命令码解析

	/***------------------*****/
	// 无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	// 写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);

	// 判断帧头数据是否为0xA5
	if (ReadFromUsart[SOF] == JUDGE_FRAME_HEADER)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(ReadFromUsart, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			;

			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(ReadFromUsart, judge_length) == TRUE)
			{
				retval_tf = TRUE; // 都校验过了则说明数据可用

				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				switch (CmdID)
				{
				case ID_game_state: // 0x0001
					memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
					break;

				case ID_game_result: // 0x0002
					memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
					break;

				case ID_game_robot_HP: // 0x0003
					memcpy(&GameRobotHP, (ReadFromUsart + DATA), LEN_game_robot_HP);
					break;

				case ID_event_data: // 0x0101
					memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;


				case ID_referee_warning: // 0x0104
					memcpy(&RefereeWarning, (ReadFromUsart + DATA), LEN_referee_warning);
					break;

				case ID_dart_remaining_time: // 0x0105
					memcpy(&DartRemainingTime, (ReadFromUsart + DATA), LEN_dart_remaining_time);
					break;

				case ID_game_robot_state: // 0x0201
					memcpy(&GameRobotState, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;

				case ID_power_heat_data: // 0x0202
					memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
					break;

				case ID_game_robot_pos: // 0x0203
					memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;

				case ID_buff_musk: // 0x0204
					memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;

				case ID_robot_hurt: // 0x0206
					memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
					break;

				case ID_shoot_data: // 0x0207
					memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
					shootNum++; // 触发一次则是发射了一颗
					// Vision_SendShootSpeed(ShootData.bullet_speed);
					break;

				case ID_bullet_remaining: // 0x0208
					memcpy(&BulletRemaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
					break;

				case ID_rfid_status: // 0x0209
					memcpy(&RfidStatus, (ReadFromUsart + DATA), LEN_rfid_status);
					break;
				case ID_sentry_status: // 0x020D
					memcpy(&SentryDecision, (ReadFromUsart + DATA), LEN_sentry_status);
					break;
				}
				// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
				if (*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					// 如果一个数据包出现了多帧数据,则再次读取
					JUDGE_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if (*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			// 如果一个数据包出现了多帧数据,则再次读取
			JUDGE_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}

	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE; // 辅助函数用
	}
	else // 只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE; // 辅助函数用
	}

	return retval_tf; // 对数据正误做处理
}

extern DMA_HandleTypeDef hdma_usart1_rx;

// 裁判系统掉线回调函数
void Judge_UartLostCallback()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1RxBuf,sizeof(usart1RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx , DMA_IT_HT);
}
// 裁判系统初始化


void JUDGE_Init()
{	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1RxBuf,sizeof(usart1RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx , DMA_IT_HT);
}


// 获取己方颜色
RobotColor JUDGE_GetSelfColor()
{
	if (JUDGE_GetSelfID() > 10) // 蓝方
	{
		return RobotColor_Blue;
	}
	else // 红方
	{
		return RobotColor_Red;
	}
}

// 获取自身ID
uint8_t JUDGE_GetSelfID()
{
	return GameRobotState.robot_id;
}

// 获取客户端ID
uint16_t JUDGE_GetClientID()
{
	return 0x100 + GameRobotState.robot_id;
}

// 获取机器人坐标
void JUDGE_GetPosition(float *x, float *y)
{
	*x = GameRobotPos.x;
	*y = GameRobotPos.y;
}

// 获取底盘功率限制
uint8_t JUDGE_GetChassisPowerLimit()
{
	return GameRobotState.chassis_power_limit;
}

// 判断发射电源是否输出
bool JUDGE_GetShooterOutputState()
{
	return GameRobotState.power_management_shooter_output;
}

bool JUDGE_GetGimbalOutputState()
{
	return GameRobotState.power_management_gimbal_output;
}

bool JUDGE_GetChassisOutputState()
{
	return GameRobotState.power_management_chassis_output;
}

// 获取枪口热量限制
uint16_t JUDGE_GetHeatLimit()
{
	return GameRobotState.shooter_barrel_heat_limit;
}

// 获取射速限制
uint16_t JUDGE_GetShootSpeedLimit()
{
	return 25;
}

// 获取底盘缓冲能量
uint16_t JUDGE_GetPowerBuffer()
{
	return PowerHeatData.chassis_power_buffer;
}

// 获取剩余枪口热量
int16_t JUDGE_GetRemainHeat()
{
	return (int16_t)GameRobotState.shooter_barrel_heat_limit - (int16_t)PowerHeatData.shooter_id1_17mm_cooling_heat;
}

// 剩余发弹数
uint16_t JUDGE_GetRemain_17_Num()
{
	return BulletRemaining.projectile_allowance_17mm;
}

// 裁判系统数据是否有效
bool JUDGE_IsValid(void)
{
	return Judge_Data_TF;
}

// 扣血原因
uint8_t HP_deduction_reason()
{
	return RobotHurt.hurt_type;
}

// 读取当前血量
uint16_t JUDGE_GetHP()
{
	return GameRobotState.current_HP;
}

// 获取冷却速度
uint16_t JUDGE_GetCoolingValue()
{
	return GameRobotState.shooter_barrel_cooling_value;
}

//获取弹速
float JUDGE_GetInitial_speed()
{
	return ShootData.initial_speed;
}
//重置挨打的装甲板id
void JUDGE_ResetHurtArmorID()
{
	RobotHurt.armor_id=0;
}
//获取挨打的装甲板id
uint8_t JUDGE_GetHurtArmorID()
{
	return RobotHurt.armor_id;
}

// 获取哨兵信息
uint32_t JUDGE_GetSentryInfo()
{
	return SentryDecision.sentry_info;
}

// 获取哨兵信息2
uint16_t JUDGE_GetSentryInfo2()
{
	return SentryDecision.sentry_info_2;
}

//获取前哨站和基地血量
uint16_t JUDGE_GetAllyOutpostHP()
{
	return GameRobotHP.ally_outpost_HP;
}

uint16_t JUDGE_GetAllyBaseHP()
{
	return GameRobotHP.ally_base_HP;
}

void Judge_Receive()
{
	JUDGE_Read_Data(usart1RxBuf);
//	memset(usart1RxBuf,0,sizeof(usart1RxBuf));
}



void Judge_Receive_update()
{
	USER_JudgeData.game_progress = GameState.game_progress;
	USER_JudgeData.remain_time = GameState.stage_remain_time;
	USER_JudgeData.current_hp = JUDGE_GetHP();
	USER_JudgeData.projectile = JUDGE_GetRemain_17_Num();

	USER_JudgeData.sentry_info = JUDGE_GetSentryInfo();
	USER_JudgeData.sentry_info_2 = JUDGE_GetSentryInfo2();
	//是否挨揍 bit0
	if (JUDGE_GetHurtArmorID()!=0&&RobotHurt.hurt_type == 0)// 被弹丸
	{
		USER_JudgeData.sentry_info_3 |= (1 << 0);
		JUDGE_ResetHurtArmorID();
	}
	else
	{
		USER_JudgeData.sentry_info_3 &=~(1 << 0);
	}
	
	// bit1 是否检测到堡垒
	if (RfidStatus.rfid_status & (1 << 17))
		USER_JudgeData.sentry_info_3 |= (1 << 1);
	else
		USER_JudgeData.sentry_info_3 &= ~(1 << 1);

	// bit2 是否检测到补给区（与兑换站不重叠）
	if (RfidStatus.rfid_status & (1 << 19))
		USER_JudgeData.sentry_info_3 |= (1 << 2);
	else
		USER_JudgeData.sentry_info_3 &= ~(1 << 2);

	// bit3 补给区（与兑换站重叠）
	if (RfidStatus.rfid_status & (1 << 20))
		USER_JudgeData.sentry_info_3 |= (1 << 3);
	else
		USER_JudgeData.sentry_info_3 &= ~(1 << 3);

	// bit4 能量 <30%
	if (BuffMusk.remaining_energy != 0x80)
	{
		if (BuffMusk.remaining_energy & (1 << 3))
			USER_JudgeData.sentry_info_3 &= ~(1 << 4);
		else
			USER_JudgeData.sentry_info_3 |= (1 << 4);
	}
	else
	{
		USER_JudgeData.sentry_info_3 &= ~(1 << 4);
	}

	// 对方前哨站增益点 bit5
	if (RfidStatus.rfid_status & (1 << 18))
		USER_JudgeData.sentry_info_3 |= (1 << 5);
	else
		USER_JudgeData.sentry_info_3 &= ~(1 << 5);

	// 对方堡垒增益点 bit6
	if (RfidStatus.rfid_status & (1 << 24))
		USER_JudgeData.sentry_info_3 |= (1 << 6);
	else
		USER_JudgeData.sentry_info_3 &= ~(1 << 6);

	USER_JudgeData.ally_outpost_hp = JUDGE_GetAllyOutpostHP();
	USER_JudgeData.ally_base_hp = JUDGE_GetAllyBaseHP();
	USER_JudgeData.shooter_barrel_cooling_value = JUDGE_GetCoolingValue();	// 获取冷却速度
	USER_JudgeData.shooter_barrel_heat_limit = JUDGE_GetHeatLimit();				//获取热量限制
	USER_JudgeData.power_management_chassis_output = JUDGE_GetChassisOutputState();
	USER_JudgeData.power_management_gimbal_output = JUDGE_GetGimbalOutputState();
	USER_JudgeData.power_management_shooter_output = JUDGE_GetShooterOutputState();
	USER_JudgeData.shooter_17mm_barrel_heat = JUDGE_GetRemainHeat(); 			//获取剩余热量
	USER_JudgeData.initial_speed = JUDGE_GetInitial_speed();				 			//获取当前弹速
	USER_JudgeData.self_color = JUDGE_GetSelfColor();

}

void Judge_Transmit_update()
{
	// 发送哨兵指令到裁判系统
	memset(&USER_CommunatianData, 0, sizeof(USER_CommunatianData));
	USER_CommunatianData.txFrameHeader.SOF = 0xA5;
	USER_CommunatianData.txFrameHeader.DataLength = sizeof(USER_CommunatianData.interactData);
	USER_CommunatianData.txFrameHeader.Seq = transmitNum;
	USER_CommunatianData.CmdID = 0x0301;
	USER_CommunatianData.interactData.data_cmd_id = 0x0120; // 哨兵机器人指令数据
	USER_CommunatianData.interactData.send_ID = JUDGE_GetSelfID();
	USER_CommunatianData.interactData.receiver_ID = 0x8080; // 0x8080为裁判系统ID
	
	SentryCmd_Pack(&USER_SentryCmd, &SentryCmd);
	memcpy(USER_CommunatianData.interactData.data,&SentryCmd.sentry_cmd,sizeof(SentryCmd.sentry_cmd));
	Append_CRC8_Check_Sum((uint8_t *)&USER_CommunatianData.txFrameHeader,sizeof(xFrameHeader));
	Append_CRC16_Check_Sum((uint8_t *)&USER_CommunatianData,sizeof(USER_CommunatianData));
	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&USER_CommunatianData,sizeof(USER_CommunatianData));
	transmitNum++;
}



void SentryCmd_Pack(USER_SentryCmd_t *user_cmd, ext_sentry_cmd_t *SentryCmd)
{
    uint32_t cmd = 0;
    /*bit0:哨兵机器人是否确认复活，0不确认复活 即使复活读条完成 1确认复活*/
    cmd |= ((uint32_t)(1 & 0x01U)) << 0;

    /*bit2~12:哨兵兑换的发弹量值*/
    cmd |= ((uint32_t)(user_cmd->buy_projectile & 0x07FFU)) << 2;

	/*bit13~16:哨兵远程兑换发弹量的请求次数*/
	cmd |= ((uint32_t)(user_cmd->remote_buy_bullet & 0x0FU)) << 13;

    /*bit17~20:哨兵远程兑换血量的请求次数*/
    cmd |= ((uint32_t)(user_cmd->remote_buy_blood & 0x0FU)) << 17;

    /*bit21~22:哨兵修改当前姿态指令 1 进攻姿态  2 防御姿态 3 移动姿态 */
    cmd |= ((uint32_t)(user_cmd->sentry_mode & 0x03U)) << 21;

    /*bit23:哨兵机器人是否确认使能机关进入正在激活状态*/
	cmd |= ((uint32_t)((user_cmd->energy_activation != 0) ? 1U : 0U)) << 23;

    SentryCmd->sentry_cmd = cmd;
}

/**********************freertos任务*********************************/
// 裁判系统发送任务回调
void Task_Judge_Callback()
{
	// if (Queue_IsEmpty(&judgeQueue))
	// 	return;
	// // 取队头的消息发送
	// JudgeTxFrame *frame = (JudgeTxFrame *)Queue_Dequeue(&judgeQueue);
	// HAL_UART_Transmit_DMA(&huart1,(uint8_t*)frame->data,frame->frameLength);
}


#ifdef EN_JUDGE_TASK
void OS_JudgeCallback(void const *argument)
{

	osDelay(500);
	for (;;)
	{
		Judge_Receive_update();
		// Judge_Transmit_update();
		// Task_Judge_Callback();
		osDelay(100);
	}
}
#endif
