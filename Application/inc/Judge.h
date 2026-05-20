#ifndef _JUDGEMENT_H_
#define _JUDGEMENT_H_

#include "main.h"
#include "stdbool.h"
#include <stdint.h>

#define    JUDGE_DATA_ERROR      0
#define    JUDGE_DATA_CORRECT    1

#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16

//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

//发送缓冲区长度
#define JUDGE_MAX_TX_LENGTH     64
//接收缓冲区长度
#define JUDGE_MAX_RX_LENGTH     1536
//发送队列长度
#define JUDGE_QUEUE_SIZE 25

//机器人颜色
typedef enum
{
	RobotColor_Red = 0,
	RobotColor_Blue = 1
}RobotColor;

typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
}JudgeFrameOffset;

//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/* 自定义帧头 */
typedef struct __attribute__((packed)) //裁判系统帧头
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} xFrameHeader;

/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  11    比赛状态数据       			发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送      
	ID: 0x0003  Byte:  16    比赛机器人血量数据   		1Hz发送    //2026.3.14修改 16byte
	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送
	ID: 0x0104	Byte: 	3		裁判警告信息				//2026.3.14修改 3byte
	ID: 0x0105	Byte: 	3		飞镖发射口倒计时		//2026.3.14修改 3byte
	ID: 0X0201  Byte: 13    机器人状态数据        		10Hz		//2026.3.14修改 13byte
	ID: 0X0202  Byte: 14    实时功率热量数据   			50Hz       
	ID: 0x0203  Byte: 12    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  8    机器人增益数据           	增益状态改变后发送
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  7    实时射击数据           		子弹发射后发送
	ID: 0x0208  Byte:  8    子弹剩余发射数					//2026.3.14修改 6byte
	ID: 0x0209  Byte:  5    机器人RFID状态					//2026.3.14修改 5byte
	ID: 0x020A  Byte:  6    飞镖机器人客户端指令数据
	ID: 0x020D Byte:  6		哨兵机器人当前状态
	ID: 0x0301  Byte:  118    机器人间交互数据           	发送方触发发送,10Hz  //2026.3.14修改 16byte
*/


//命令码ID,用来判断接收的是什么数据
typedef enum
{
	ID_game_state       				= 0x0001,//比赛状态数据
	ID_game_result 	   					= 0x0002,//比赛结果数据
	ID_game_robot_HP      			= 0x0003,//比赛机器人血量数据
	ID_event_data  							= 0x0101,//场地事件数据 *
	ID_referee_warning					= 0x0104,//裁判警告信息
	ID_dart_remaining_time			= 0x0105,//飞镖发射口倒计时
	ID_game_robot_state    			= 0x0201,//机器人状态数据  *
	ID_power_heat_data    			= 0x0202,//实时功率热量数据
	ID_game_robot_pos        		= 0x0203,//机器人位置数据  *
	ID_buff_musk								= 0x0204,//机器人增益数据//
	ID_robot_hurt								= 0x0206,//伤害状态数据
	ID_shoot_data								= 0x0207,//实时射击数据
	ID_bullet_remaining					= 0x0208,//子弹剩余发射数
	ID_rfid_status							= 0x0209,//机器人RFID状态  *	
	ID_sentry_status          	= 0x020D,
} CmdID;



//命令码数据段长,根据官方协议来定义长度
typedef enum
{
	LEN_game_state       					= 11,	//0x0001
	LEN_game_result       				= 1,	//0x0002
	LEN_game_robot_HP							= 16,	//0x0003
	LEN_event_data  							= 4,	//0x0101
	LEN_referee_warning						= 3,	//0x0104
	LEN_dart_remaining_time				= 3,	//0x0105
	LEN_game_robot_state    			= 13,	//0x0201
	LEN_power_heat_data   				= 14,	//0x0202
	LEN_game_robot_pos        		= 12,	//0x0203  
	LEN_buff_musk        					= 8,	//0x0204
	LEN_robot_hurt        				= 1,	//0x0206
	LEN_shoot_data       					= 7,	//0x0207
	LEN_bullet_remaining					= 6,	//0x0208
	LEN_rfid_status								= 5,	//0x0209
	LEN_sentry_status   					= 6,  //0x020D
} JudgeDataLength;


////机器人交互信息
//typedef struct __attribute__((packed))

//{
//	xFrameHeader   							txFrameHeader;//帧头
//	uint16_t								CmdID;//命令码
//	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
//	robot_interactive_data_t  	 			interactData;//数据段
//	uint16_t		 						FrameTail;//帧尾
//}ext_CommunatianData_t;


/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef struct __attribute__((packed))
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct __attribute__((packed))
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */   //2026.3.14 改动
typedef struct __attribute__((packed))
{
	uint16_t ally_1_robot_HP;
	uint16_t ally_2_robot_HP;
	uint16_t ally_3_robot_HP;
	uint16_t ally_4_robot_HP;
	uint16_t reserved;
	uint16_t ally_7_robot_HP;
	uint16_t ally_outpost_HP;
	uint16_t ally_base_HP;
} ext_game_robot_HP_t;
 
/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct __attribute__((packed)) 					//2026.3.14修改
{ 	
	uint32_t event_data;
	
	//以下是队内自定义结构体 
	//0：未占领/未激活  1：已占领/已激活 
	/*bit 0-2： 
		bit 0：己方与兑换区不重叠的补给区占领状态，1为已占领 
		bit 1：己方与兑换区重叠的补给区占领状态，1为已占领 
		bit 2：己方补给区的占领状态，1为已占领（仅 RMUL 适用） 
		bit 3-6：己方能量机关状态 
		bit 3-4：己方小能量机关的激活状态，0为未激活，1为已激活，2为正在激活
		bit 5-6：己方大能量机关的激活状态，0为未激活，1为已激活，2为正在激活
		bit 7-8：己方中央高地的占领状态，1为被己方占领，2为被对方占领 
		bit 9-10：己方梯形高地的占领状态，1为已占领 
		bit 11-19：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0） 
		bit 20-22：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，
							 1为击中前哨站，2为击中基地固定目标，3为击中基地随机
							 固定目标，4为击中基地随机移动目标 5为击中基地末端移动目标
		bit 23-24：中心增益点的占领状态，0为未被占领，1为被己方占领，2
							 为被对方占领，3为被双方占领。（仅RMUL适用） 
		bit 25-26：己方堡垒增益点的占领状态，0为未被占领，1为被己方占
							 领，2为被对方占领，3为被双方占领。
		bit 27-28：己方前哨站增益点的占领状态， 0 为未被占领， 1 为被己方
							 占领， 2 为被对方占领
		bit 29：己方基地增益点的占领状态， 1 为已占领							 */
	
//	uint8_t supply_area_state : 3;
//	uint8_t own_buff_state : 4;
//	uint8_t own_central_land_state : 2;
//	uint8_t own_trapezoidal_land_state : 2;
//	uint16_t darts_hitted_time : 9;
//	uint8_t darts_hitted_target : 3;
//	uint8_t center_gain_state : 2;
//	uint8_t own_fortress_state :2;
//	uint8_t own_outpost_state : 2;   // bit27-28 己方前哨站增益点状态
//	uint8_t own_base_state : 1;      // bit29 己方基地增益点状态
//	
//	uint32_t reserved : 2;
	
} ext_event_data_t; 



/* ID: 0x104    Byte: 3    裁判警告信息 */
typedef struct __attribute__((packed))
{
	uint8_t level; 
	uint8_t offending_robot_id; 
	uint8_t count; 
} ext_referee_warning_t;

/* ID: 0x105    Byte: 3    飞镖发射口倒计时 */  
typedef struct __attribute__((packed))
{ 
	uint8_t dart_remaining_time; 
	uint16_t dart_info; 
}ext_dart_info_t; 

/* ID: 0X0201  Byte: 13    机器人状态数据 */
typedef struct __attribute__((packed))
{
	uint8_t robot_id; 
	uint8_t robot_level; 
	uint16_t current_HP;  
	uint16_t maximum_HP; 
	uint16_t shooter_barrel_cooling_value; 
	uint16_t shooter_barrel_heat_limit; 
	uint16_t chassis_power_limit;  
	uint8_t power_management_gimbal_output : 1; 
	uint8_t power_management_chassis_output : 1;  
	uint8_t power_management_shooter_output : 1;
} ext_game_robot_status_t;


/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef struct __attribute__((packed))
{
	uint16_t reserved_1;
	uint16_t reserved_2;
	float reserved_3;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;


/* ID: 0x0203  Byte: 12    机器人位置数据 */
typedef struct __attribute__((packed))
{ 
	float x; 
	float y; 
	float angle; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  8    机器人增益数据 */
typedef struct __attribute__((packed))

{ 
	uint8_t recovery_buff;  
	uint16_t cooling_buff;  
	uint8_t defence_buff;  
	uint8_t vulnerability_buff; 
	uint16_t attack_buff; 
	uint8_t remaining_energy; 
} ext_buff_musk_t; 


/* ID: 0x0206  Byte:  2    伤害状态数据 */
typedef struct __attribute__((packed))

{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct __attribute__((packed))

{
	uint8_t bullet_type;  
	uint8_t shooter_number; 
	uint8_t launching_frequency;  
	float initial_speed; 
} ext_shoot_data_t;

/* ID: 0x0208  Byte:  8   子弹剩余发射数 */
typedef struct __attribute__((packed))

{
	uint16_t projectile_allowance_17mm; 
	uint16_t projectile_allowance_42mm;  
	uint16_t remaining_gold_coin; 
	uint16_t projectile_allowance_fortress;
} ext_bullet_remaining_t;

/* ID: 0x0209  Byte:  5    机器人RFID状态 */ //2026.3.14修改
typedef struct __attribute__((packed))

{
	uint32_t rfid_status;
	uint8_t rfid_status_2;
} ext_rfid_status_t;

/* ID: 0x020A  Byte:  6    飞镖机器人客户端指令数据 */
typedef struct __attribute__((packed))

{
	uint8_t dart_launch_opening_status;  
	uint8_t reserved;  
	uint16_t target_change_time;  
	uint16_t latest_launch_cmd_time; 
} ext_dart_client_cmd_t;	

/* ID: 0x020D  Byte:  6    哨兵机器人当前状态数据 */
typedef struct __attribute__((packed))
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
}ext_sentry_info_t;

/* ID: 0x0303  Byte: 12  选手端发送地图位置数据 */
typedef struct __attribute__((packed))
{
	float target_position_x;  	//目标x轴位置
	float target_position_y;  	//目标y轴位置
	uint8_t cmd_keyboard;		//云台手按下键盘通用键值 无则为0
	uint8_t target_robot_id;	//目标机器人ID 发送坐标数据时为0
	uint16_t cmd_source;		//信息来源的ID 0x0106为红方云台手选手端 ，0x016A为蓝方云台手选手端，其余为其他来源
}map_command_t;

/*以下为通过裁判系统给其他机器人发送/接受 以及给裁判系统发送数据*/
/* 
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	11，英雄(蓝)；
	12，工程(蓝)；
	13/14/15，步兵(蓝)；
	16，空中(蓝)；
	17，哨兵(蓝)。 
	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)； 
	0x0111，英雄操作手客户端(蓝)；
	0x0112，工程操作手客户端(蓝)；
	0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
	0x0116，空中操作手客户端(蓝)。 
*/

/* 机器人交互数据接收信息：0x0301  */
/* 
	学生机器人间通信 cmd_id 0x0301，子内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			备注 
	0 			2 		子内容 ID 		0x0200~0x02FF  
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 机器人之间通信 
										0x0100 选手端删除图层 0x0101-0x0105 UI画图形 0x0110 UI画字符图形 0x120 哨兵自主决策 0x121 雷达自主决策
	
	2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性， 机器人ID编号如下所示：

	4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID 
	ID如下：
	1：红方英雄机器人	2：红方工程机器人	3/4/5：红方步兵机器人（与机器人ID3~5对应）		6：红方空中机器人	7：红方哨兵机器人	8：红方飞镖   9：红方雷达	10：红方前哨站	11：红方基地
	101：蓝方英雄机器人	102：蓝方工程机器人	103/104/105：蓝方步兵机器人（与机器人ID3~5对应）106：蓝方空中机器人	107：蓝方哨兵机器人	108：蓝方飞镖 109：蓝方雷达	110：蓝方前哨站	111：蓝方基地
	0x8080 裁判系统服务器

	6 			n 		数据段 			n 需要小于 112

*/
typedef struct __attribute__((packed))
{
	uint16_t data_cmd_id;
	uint16_t send_ID;
	uint16_t receiver_ID;
	uint8_t data[4];
} robot_interactive_data_t;  //很丑陋的烧饼专用data[4]

//机器人交互信息
typedef struct __attribute__((packed))

{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	robot_interactive_data_t  	 			interactData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_CommunatianData_t;

/*以下为字内容ID*/
/* ID: 0x0120  Byte: 4  哨兵机器人当前状态数据 */	
typedef struct __attribute__((packed))
{
	uint32_t sentry_cmd;
}ext_sentry_cmd_t;

/*字内容ID结束*/
typedef struct
{
	uint8_t data[JUDGE_MAX_TX_LENGTH];
	uint16_t frameLength;
}JudgeTxFrame;


/*以下为自定义裁判系统内容*/
typedef struct __attribute__((packed))
{
	uint8_t game_progress; // 当前比赛状态 0:未开始比赛 1:准备阶段 2:自检阶段 3:五秒倒计时 4:比赛中 5:比赛结算中
	uint16_t remain_time;  // 比赛剩余时间 单位:s
	uint16_t current_hp;   // 当前血量
	uint16_t projectile;   // 弹仓还剩多少蛋
	uint32_t sentry_info; 	//bit0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为0，在哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的允许发弹量值
							//bit11-14：哨兵机器人成功远程兑换允许发弹量的次数，开局为0，在哨兵机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远程兑换允许发弹量的次数
							//bit15-18：哨兵机器人成功远程兑换血量的次数，开局为0，在哨兵机器人成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的次数
							//bit19：哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为1，否则为0
							//bit20：哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为1，否则为0
							//bit21-30：哨兵机器人当前若兑换立即复活需要花费的金币数。
							//bit31 保留
	uint16_t sentry_info_2;	// bit0：哨兵当前是否处于脱战状态，处于脱战状态时为1，否则为0
							// bit1-11：队伍17mm允许发弹量的剩余可兑换数
							// bit12-13:哨兵当前姿态，1为进攻姿态，2为防御姿态，3为移动姿态
							// bit14：己方能量机关是否能够进入正在激活状态，1为当前可激活
							// bit 15：保留位

	uint8_t sentry_info_3;  // bit 0: 装甲板是否被攻击 0:否 1:是
							// bit 1: RFID 是否检测到堡垒 0:否 1:是
							// bit 2: RFID 是否检测到补给区(与兑换站不重叠) 0:否 1:是
							// bit 3: RFID 是否检测到补给区(与兑换站重叠) 0:否 1:是							
							// bit 4: 当前剩余能量值是否小于30% 0:否 1:是

	uint16_t ally_outpost_hp;  // 己方前哨站血量
	uint16_t ally_base_hp;     // 己方基地血量
/***********以上为发给ai的**************/
	uint16_t shooter_barrel_cooling_value;  // 冷却速度
	uint16_t shooter_barrel_heat_limit; 		//热量限制
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
	uint16_t shooter_17mm_barrel_heat;			//剩余热量
	float initial_speed;										//弹速
	uint8_t self_color;
}JudgeData_t;


/*哨兵自主决策指令协议详细内容*/
/*	bit0 0为确认不复活 即使读条完毕 1为确认复活 读条后立马复活
	bit1 0为不买活 1为买活
	bit2-12 哨兵回家买蛋买多少 开局为0 这个值只能单增
	bit13-16 哨兵远程买蛋次数 开局为0 改这个就能买蛋 0到1买一次 1到2买一次 依此类推
	bit17-20 哨兵远程买血次数 开局为0 改这个就能买血 0到1买一次 1到2买一次 依此类推
	bit21-22 哨兵姿态切换 1进攻 2防御 3移动 默认为3移动 修改就能变姿态
	bit23 是否确认开符 0为不开 1为确认开
	bit24-31 保留
*/


typedef struct __attribute__((packed))
{
	uint8_t sentry_mode; //1为进攻 2为防守 3为移动 默认为3
	uint8_t energy_activation; //0为不激活 1为激活小符 2为激活大符 激活小还是大和比赛开始时间有关
	uint16_t buy_projectile; //哨兵要买多少发弹 开局为0 修改后烧饼在补血点就能兑换 只能单增 如0->100买100发 100->101买1发 依此类推
	uint8_t buy_life; 	//0为不买活 1为买活
	uint8_t remote_buy_blood;	//0到1为买一次 1到2为买一次 依此类推
	uint8_t remote_buy_bullet; //0到1为买一次 1到2为买一次 依此类推
}USER_SentryCmd_t; //哨兵机器人指令数据

/****************函数声明***************/
void JUDGE_Init(void);
bool JUDGE_Read_Data(uint8_t *ReadFromUsart);
RobotColor JUDGE_GetSelfColor(void);
uint8_t JUDGE_GetSelfID(void);
uint16_t JUDGE_GetClientID(void);
bool JUDGE_IsValid(void);
void JUDGE_GetPosition(float *x,float *y);
uint8_t JUDGE_GetChassisPowerLimit(void);
bool JUDGE_GetShooterOutputState(void);
bool JUDGE_GetGimbalOutputState(void);
uint16_t JUDGE_GetHeatLimit(void);
uint16_t JUDGE_GetShootSpeedLimit(void);
uint16_t JUDGE_GetPowerBuffer(void);
int16_t JUDGE_GetRemainHeat(void);
uint16_t JUDGE_GetRemain_42_Num(void);
uint8_t HP_deduction_reason(void);
uint16_t JUDGE_GetHP(void);
uint16_t JUDGE_GetCoolingValue(void);


//串口6中断回调
void USER_USART1_IRQHandler(void);
//任务回调
void Task_Judge_Callback(void);
//掉线回调
void Judge_UartLostCallback(void);

void Judge_Receive(void);

void SentryCmd_Pack(USER_SentryCmd_t *user_cmd, ext_sentry_cmd_t *SentryCmd);

/****************外部引用***************/
extern ext_game_robot_pos_t			GameRobotPos;
extern ext_shoot_data_t				ShootData;
extern ext_game_robot_status_t		GameRobotState;
extern JudgeData_t					USER_JudgeData;
extern USER_SentryCmd_t				USER_SentryCmd;
extern ext_power_heat_data_t		PowerHeatData;

#endif //头文件
