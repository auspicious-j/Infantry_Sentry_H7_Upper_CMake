#ifndef _SHOOTER_H_
#define _SHOOTER_H_

#include "Moto.h"
#include "stdbool.h"
#include "Slope.h"
#include "USER_Moto.h"
#include "USER_RC.h"

enum
{
	IDLE = 0,
	TRIGGER,
	TRIGGER_REVERSE,
	TRIGGER_CONTINUE,
	TRIGGER_DOUBLE,
	TRIGGER_CLICK,
};

typedef struct
{
	int number;				  // 发弹量
	uint8_t workState;		  // 工作状态
	
	DJI_Motor_t triggerMotor; // 拨弹电机和摩擦轮电机
	DJI_Motor_t fricMotor[2];
	
	bool fricOpenFlag;	  // 摩擦轮开启标志  0开1关

	int16_t fricSpd;	  // 摩擦轮速度
	Slope fricSlope;	  // 摩擦轮斜坡
	float ave_bullet_speed;
	float bullet_speed;

	float distance;		 // 测距 用来补偿
	float last_bullet_speed;
	uint8_t box;
	
	struct
	{
		uint16_t judgeCnt, reverseCnt; // 堵转判定计数器,反转计数器
		uint16_t fric_judgeCnt, fric_reverseCnt;
		_Bool state;
	} block; // 堵转处理相关数据

} Shooter;

extern Shooter shooter;

void Shooter_Init(void);
void Shooter_state(_Bool openflag);
void Shooter_RegisterEvents();
void Shooter_SwitchState_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_StartFric_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_StopFric_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_IncFricSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
void Shooter_DecFricSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);

#endif
