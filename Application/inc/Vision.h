#ifndef _VISION_H_
#define _VISION_H_

#include "main.h"
#include "stdint.h"
#include "stdbool.h"

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
	uint16_t projectile;   // 弹仓还剩多少蛋
	uint8_t sentry_info;   // bit 0: 装甲板是否被攻击 0:否 1:是
											 	 // bit 1: 是否脱战 0:否 1:是
												 // bit 2: RFID 是否检测到堡垒 0:否 1:是
												 // bit 3: RFID 是否检测到补给区(与兑换站不重叠) 0:否 1:是
											   // bit 4: RFID 是否检测到补给区(与兑换站重叠) 0:否 1:是
												 // bit 5: 当前剩余能量值是否小于30% 0:否 1:是
												 // bit 6-7: 0
												 // building state
	uint16_t red_outpost_hp;  // 红方前哨站血量
	uint16_t red_base_hp;     // 红方基地血量
	uint16_t blue_outpost_hp; // 蓝方前哨站血量
	uint16_t blue_base_hp;    // 蓝方基地血量

}Judge_Data_e;

typedef struct __attribute__((packed))
{
	uint8_t header;
	uint8_t detect_color;  // 0-red 1-blue
	
	float roll;
	float pitch;
	float top_yaw;
	float diff_yaw;//大小yaw之间相差角度
  float diff_pitch;//大小yaw之间pitch差值
	float bullet_speed;
	uint8_t robo_status; //敌方机器人死没死
	
	float yaw_delta;  //大yaw与底盘相对角度
	
	Judge_Data_e AI_Judge_data;
	
 	uint8_t end_frame;   
	//uint16_t checksum;
}VisionTransmit;

typedef struct __attribute__((packed))
{
	uint8_t header;
	
	int8_t tracking; //是否正在瞄准
	
	float base_yaw;  //自瞄目标角度 单位°
	float top_yaw;	 //自瞄目标角度 单位°
	float pitch;
	
	float armor_yaw;//单位弧度 （目标角度和目前角度差值，用于火控）
	float distance;//两车中心距离
	float armor_radius;	//装甲板的物理半径
	
	uint8_t force_shoot;//检测到 强制开一发火 0是不开 1为开火
	
/***********以下为ai传输内容***********/
	float linear_x;
	float linear_y;	 
	float angular_z; //旋转速度
  float fake_relativeAngle; //假云台的relative angle 用于电控小陀螺
	
	uint8_t spin_mode;  //0为小陀螺关  1为小陀螺开
	
	uint8_t end_frame;   
	//uint16_t checksum;
}VisionReceive;

typedef struct
{	
	float base_yaw;
	float top_yaw;
	float pitch;
	uint8_t mode;
	unsigned char found;
	float fire;
	float v_yaw;
	float distance;//两车中心距离
	float distance_to_center;//云台中心到锁定装甲板中心距离
	float yaw_slope;
	float pitch_slope;
}Vision_t;

extern VisionReceive vision_receive;
extern VisionTransmit vision_transmit;
extern Vision_t vision;

void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length);
void Vision_DataTransmit(void);
void Vision_DataUpdate(void);
void Vision_Init(void);
void Vision_ParseData(void);

#endif
