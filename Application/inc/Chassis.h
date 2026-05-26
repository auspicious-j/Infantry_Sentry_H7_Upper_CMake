#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "main.h"
#include "pid.h"
#include "Slope.h"
#include "USER_Moto.h"
#include "USER_RC.h"
#include <stdint.h>

typedef enum
{
	ChassisMode_Follow,	  // 底盘跟随云台模式
	ChassisMode_Spin,	  // 小陀螺模式
} Chassis_Mode_e;

typedef enum{
    Chassis_control,
}Chassis_Pattern_e;

typedef struct _Chassis
{
	// 底盘尺寸信息
	struct Info
	{
		float wheelbase;	// 轴距
		float wheeltrack;	// 轮距
		float wheelRadius;	// 轮半径
		float offsetX;		// 重心在xy轴上的偏移
		float offsetY;
		float R;			// 轮子到中心的距离
		float rpm_ratio;	// 电机转速转线速度的换算系数
	} info;
	// 4个电机
	DJI_Motor_t motors[4];
	// 底盘移动信息
	struct Move
	{
		float vx; // 当前左右平移速度 mm/s
		float vy; // 当前前后移动速度 mm/s
		float vw; // 当前旋转速度 rad/s

		float maxVx, maxVy, maxVw; // 三个分量最大速度

		float real_vx;//根据当前轮速解算实际车速
		float real_vy;
		float real_vw;
		PID real_xPID, real_yPID, real_wPID; // 速度pid

		float maxPower;
		Slope xSlope, ySlope, outputSlope, chargeSlope, spinSlope; // 斜坡
		uint8_t fastMode; // 快速模式
	} move;
	
	struct
	{
		int8_t key_w;
		int8_t key_a;
		int8_t key_s;
		int8_t key_d;
	} key;

	// 旋转相关信息
	struct
	{
		PID pid;				// 旋转PID，由relativeAngle计算底盘旋转速度
		float relativeAngle;	// 云台与底盘的偏离角 单位度
		float InitAngle;		// 云台与底盘对齐时的编码器度数 
		int16_t InitpitchAngle; // 云台水平时编码器值
		float nowAngle;		// 此时云台的编码器换算为°值
		Chassis_Mode_e mode;		// 底盘模式 小陀螺或者底盘跟随
		float ratio;				// 旋转速度系数 占最大速度的多少
	} rotate;
	struct
	{
		float pitchTiltAngle; //底盘和地面的倾斜角
		float rollTiltAngle;
	}angle;
	Chassis_Pattern_e pattern;
} Chassis_t;

extern Chassis_t chassis;
extern float vx,vy,vw;
extern uint16_t SET_WHEELSPEED_MAX;

void Chassis_InitPID(void);
void Chassis_UpdateSlope();
void Chassis_RegisterEvents();
void Chassis_Move_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void UI_UPdate(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_SwitchMode_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_Stop_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Chassis_capOutputChange_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);

void Gimbal_RegisterEvents();


#endif
