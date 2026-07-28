#ifndef _VISION_H_
#define _VISION_H_

#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "USER_RC.h"

#define    VISION_FRAME_HEADER_TX  	0x5A
#define    VISION_FRAME_HEADER_RX  	0xA5

#ifndef PI
#define PI 3.14159265f
#endif

typedef struct __attribute__((packed))
{
	uint8_t game_progress; // 当前比赛状态 0:未开始比赛 1:准备阶段 2:自检阶段 3:五秒倒计时 4:比赛中 5:比赛结算中
	uint16_t remain_time;  // 比赛剩余时间 单位:s
	uint16_t current_hp;   // 当前血量
	uint16_t projectile;    // 哨兵当前剩余允许发弹量

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
							// bit 5：RFID 是否检测到对方前哨站增益点
							// bit 6：RFID 是否检测到对方堡垒增益点
	uint8_t rune_state;     // bit 0-1:小符的状态 0未激活 1已激活 2正在激活
							// bit 2-3:大符的状态 0未激活 1已激活 2正在激活
	uint16_t ally_outpost_hp;  // 己方前哨站血量
	uint16_t ally_base_hp;     // 己方基地血量
}Judge_Data_e;

typedef struct __attribute__((packed))
{
  uint8_t header ;
  //步兵模式下的自瞄数据
//  uint8_t control;      // 自瞄是否控制云台 0 不控制 1 控制 //
//  float fire_thres_yaw; // 火控阈值
//  float fire_thres_pitch;
//  float target_yaw; // 目标yaw角度
//  float target_pitch; // 目标pitch角度
//  float yaw; // 云台角度、速度、加速度(弧度制,直接发,不要乘1000)
//  float yaw_vel;
//  float yaw_acc;
//  float pitch;
//  float pitch_vel;
//  float pitch_acc;
//  uint32_t bullet_id; // 自增的子弹ID
//  uint16_t checksum ;
	
	float linear_x;
	float linear_y;	 
	float angular_z; //旋转速度
	
	
	uint8_t tracking; //0表示没瞄到 1表示瞄到装甲板 2表示瞄到符
	
	float base_yaw;  //自瞄目标角度 单位°
	float top_yaw;	 //自瞄目标角度 单位°
	float pitch;
	
	float incident_yaw;//单位弧度 （目标角度和目前角度差值，用于火控）
	float distance;//两车中心距离
	float armor_radius;	//装甲板的物理半径

	uint8_t rune_number;  //打符模式使用 变化就打弹 打一发后没变化 0.5秒后再打一发

	uint8_t force_shoot;//检测到 强制开一发火 0是不开 1为开火
	
/***********以下为ai传输内容***********/
	
	
	float align_yaw; //和起伏路段对齐角度
	float rune_yaw; //符的角度
	float outpost_yaw; //前哨站的角度

	uint8_t spin_mode;  //0为小陀螺开  1为小陀螺关
	uint8_t sentry_mode; //1为进攻 2为防守 3为移动 默认为3
	uint8_t armor_mode;  //0为打车 1打前哨 2为打符
	uint8_t align_mode;  //是否对齐 0为不对齐 1为对齐装甲板
	uint8_t energy_activation; //0为不激活 1为激活小符 2为激活大符 激活小还是大和比赛开始时间有关
	uint8_t buy_life; 	//0为不买活 1为买活
	uint8_t remote_buy_blood;	//0到1为远程买一次血 1到2为买一次 依此类推 
	uint8_t remote_buy_bullet;  //0到1为远程买一次弹 1到2为买一次 依此类推
	uint16_t buy_projectile; //哨兵要买多少发弹 开局为0 修改后烧饼在补血点就能兑换 只能单增 如0->100买100发 100->101买1发 依此类推
	uint8_t end_frame;   

}VisionReceive;

typedef struct __attribute__((packed))
{
	//步兵模式下发送
//	uint8_t header;//0x5A
//	uint8_t task_mode;   // 当前自瞄模式 0 空闲 1 打装甲板 2 小符 3 大符
//	uint8_t enemy_color; // 敌人颜色 0 红色 1 蓝色
//	float bullet_speed;  // 弹速
//	float roll;          // 云台的外旋rpy角和角速度(弧度制，直接发，不要乘1000)
//	float pitch;
//	float pitch_vel;
//	float yaw;
//	float yaw_vel;
//	uint32_t bullet_id; // 打出子弹时刻返回的子弹ID（目前没用上）
//	uint16_t checksum ;
	uint8_t header;
	uint8_t detect_color;  // 0-red 1-blue
	uint8_t mode;  //0为打装甲板 1为打符
	
	float roll;
	float pitch;
	float top_yaw;
	float diff_yaw;//大小yaw之间相差角度
  	float diff_pitch;//大小yaw之间pitch差值
	float bullet_speed;
	uint8_t robo_status; //敌方机器人死没死
	

	Judge_Data_e AI_Judge_data;
	uint8_t see_enemy; //0表示没瞄到 1表示瞄到装甲板 2表示瞄到符
	
 	uint8_t end_frame;   
	//uint16_t checksum;
}VisionTransmit;

typedef struct __attribute__((packed))
{
	float yaw;
	float pitch;
	uint8_t fire;
	uint8_t found;
} VisionSensorInfo;

typedef struct vision_sensor_struct {
	VisionReceive		*info;
	VisionTransmit		*transmit_info;
	VisionSensorInfo    *sent_info;
	void				(*Init)(void);
	void				(*Update)(void);
    void                (*DataReceive)(uint8_t *read_from_usart, uint32_t length);
	void                (*Data_Transmit)(void);
} VisionSensor;

typedef struct
{	
	//步兵模式下发送
//	uint8_t control; // 自瞄是否控制云台 0 不控制 1 控制
//	float fire_thres_yaw; // 火控阈值
//	float fire_thres_pitch;
//	float target_top_yaw; //原始数据 用于火控
//	float target_top_pitch;
//	float top_yaw;       // 用于电控云台控制 经过mpc优化
//	float top_yaw_vel;
//	float top_yaw_acc;
//	float top_pitch;
//	float top_pitch_vel;
//	float top_pitch_acc;
//	uint32_t bullet_id; // 自增的子弹ID
//	uint16_t checksum ;
//	uint8_t mode;
	float base_yaw;
	float top_yaw;
	float pitch;
	uint8_t mode;
	uint8_t tracking;
	float fire;
	float v_yaw;
	float distance;//两车中心距离
	float distance_to_center;//云台中心到锁定装甲板中心距离
	float yaw_slope;
	float pitch_slope;
}Vision_Type;



extern VisionReceive vision_receive;
extern VisionTransmit vision_transmit;
extern Vision_Type vision;
extern uint8_t Vision_Mode;

void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length);
void Vision_DataTransmit(void);
void Vision_DataUpdate(void);
void Vision_Init(void);
void Vision_ParseData(void);
void Vision_RegisterEvents();
void Vision_Change_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);

#endif
