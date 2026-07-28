#include "Shooter.h"
#include "Judge.h"
#include "USER_RC.h"
#include "bsp_can.h"
#include "Slope.h"
#include <stdio.h>
#include <math.h>
#include "chassis.h"
#include "beep.h"
#include "shooter.h"
#include "vision.h"
#include "gimbal.h"
#include "judge.h"

Shooter shooter;
uint8_t shootMaxSpeed = 24;
uint32_t t = 0; // 线性火控延时
int8_t shootSpeed = -1;


void Shooter_InitPID(void);


//射击系统初始化
void Shooter_Init()
{
	shooter.fricSpd = 5800;					//
	Slope_Init(&shooter.fricSlope, 140, 0); // 摩擦轮斜坡
	Shooter_InitPID();						// m初始化电机pid
	Shooter_RegisterEvents();				// 注册事件
	shooter.fricMotor[0].targetSpeed = -0;
	shooter.fricMotor[1].targetSpeed = +0;
	shooter.workState = IDLE;
}

void Shooter_InitPID()
{
	Motor_StartCalcAngle(&shooter.triggerMotor);							 // 初始化电机角度累计
	PID_Init(&shooter.triggerMotor.anglePID.inner,3.5,0,0,7800,10000);
	PID_Init(&shooter.triggerMotor.anglePID.outer, 0.2,0,0.01, 0, 6000);

	PID_Init(&shooter.fricMotor[0].speedPID, 25, 0, 5, 0, 16000); // 摩擦轮
	PID_Init(&shooter.fricMotor[1].speedPID, 25, 0, 5, 0, 16000);
	SMCInit(&shooter.fricMotor[0].FricSMC, 0.03682, 205.121796, 2.62, 27.417, 5.0); // 左
	SMCInit(&shooter.fricMotor[1].FricSMC, 0.03682, 205.121796, 2.62, 27.417, 5.0); // 右
}

/*************************RC事件**************************
以下任务受键鼠event调度
*********************************************************/
// 注册事件--射击

void Shooter_RegisterEvents()
{
	// 左键按下抬起开关拨弹
	RC_Register(Key_Left, CombineKey_None, KeyEvent_OnClick, Shooter_SwitchState_KeyCallback);
	RC_Register(Key_Left, CombineKey_None, KeyEvent_OnUp, Shooter_SwitchState_KeyCallback);
	RC_Register(Key_Left, CombineKey_None, KeyEvent_OnLongPress, Shooter_SwitchState_KeyCallback);
	// F开启摩擦轮
	RC_Register(Key_F, CombineKey_None, KeyEvent_OnDown, Shooter_StartFric_KeyCallback);
	//G关闭摩擦轮
	RC_Register(Key_G,CombineKey_None,KeyEvent_OnDown,Shooter_StopFric_KeyCallback);
	// SHIFT+Q提高100摩擦轮转速
	RC_Register(Key_Q, CombineKey_Shift, KeyEvent_OnDown, Shooter_IncFricSpeed_KeyCallback);
	// SHIFT+E减小100摩擦轮转速
	RC_Register(Key_E, CombineKey_Shift, KeyEvent_OnDown, Shooter_DecFricSpeed_KeyCallback);
}

// 触发/停止拨弹工作
void Shooter_SwitchState_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (event)
	{
	case KeyEvent_OnClick:			   // 单发拨弹
		if (shooter.fricOpenFlag == 1) // 摩擦轮开启 允许拨弹
		{
			shooter.workState = TRIGGER_CLICK;
		}
		break;

	case KeyEvent_OnLongPress:		   // 连发拨弹
		if (shooter.fricOpenFlag == 1) // 摩擦轮开启 允许拨弹
		{
			shooter.workState = TRIGGER_CONTINUE;
		}
		break;

	case KeyEvent_OnUp:				   // 鼠标左键抬起
		if (shooter.fricOpenFlag == 1) // 摩擦轮开启 ，不允许拨弹
		{
			shooter.workState = IDLE;
		}
		break;
	default:
		break;
	}
}

// 手动打开/关闭摩擦轮
void Shooter_StartFric_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	shooter.fricMotor[0].targetSpeed = -shooter.fricSpd;
	shooter.fricMotor[1].targetSpeed = shooter.fricSpd;
	shooter.fricOpenFlag=1;
}

void Shooter_StopFric_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	shooter.fricOpenFlag=0;
	shooter.fricMotor[0].targetSpeed = 0;
	shooter.fricMotor[1].targetSpeed = 0;
}

void Shooter_IncFricSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	shooter.fricSpd += 100;
	Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd); // 摩擦轮斜坡
	Slope_NextVal(&shooter.fricSlope);					  // 斜坡下一个值
	shooter.fricMotor[0].targetSpeed = -Slope_GetVal(&shooter.fricSlope)*shootSpeed;
	shooter.fricMotor[1].targetSpeed = +Slope_GetVal(&shooter.fricSlope)*shootSpeed;
}

void Shooter_DecFricSpeed_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	shooter.fricSpd -= 100;
	Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd); // 摩擦轮斜坡
	Slope_NextVal(&shooter.fricSlope);					  // 斜坡下一个值
	shooter.fricMotor[0].targetSpeed = -Slope_GetVal(&shooter.fricSlope)*shootSpeed;
	shooter.fricMotor[1].targetSpeed = +Slope_GetVal(&shooter.fricSlope)*shootSpeed;
}
uint32_t getCurrentMicros(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	uint32_t m = HAL_GetTick();
	__IO uint32_t v = SysTick->VAL;
	// If an overflow happened since we disabled irqs, it cannot have been
	// processed yet, so increment m and reload VAL to ensure we get the
	// post-overflow value.
	if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
	{
		++m;
		v = SysTick->VAL;
	}

	// Restore irq status
	__set_PRIMASK(primask);

	const uint32_t tms = SysTick->LOAD + 1;
	return (m * 1000 + ((tms - v) * 1000) / tms);
}
/******************结束键鼠*******************/

bool Heat_Limit()
{
	float d = 0.6; // 阈值
	(void)d;	   // 阈值
	float k = 1;   // 防止浪费未使用热量,增加冷却缩减系数
	if (JUDGE_GetRemainHeat() < 20)
	{
		k = 5;
		return true;
	}
	else if (JUDGE_GetRemainHeat() <= 35)
	{
		k = 1;
		t = k * 1000 * 10 / JUDGE_GetCoolingValue() * 1.01f;
		return true;
	}
	else if (JUDGE_GetRemainHeat() <= 100)
	{
		k = 0.5;
		t = k * 1000 * 10 * (-JUDGE_GetRemainHeat() + 100) / (100 - 30) / JUDGE_GetCoolingValue();

		return true;
	}
	else
	{
		t = 0;
		k = 0.5;
		return true;
	}
}



void Shooter_state(_Bool openflag)
{
	if (openflag == 1)
	{
		Slope_SetTarget(&shooter.fricSlope, shooter.fricSpd);				  // 摩擦轮斜坡
		Slope_NextVal(&shooter.fricSlope);									  // 斜坡下一个值
		shooter.fricMotor[0].targetSpeed = -Slope_GetVal(&shooter.fricSlope); // 摩擦轮速度
		shooter.fricMotor[1].targetSpeed = Slope_GetVal(&shooter.fricSlope);
	}
	else
	{
		Slope_SetTarget(&shooter.fricSlope, 0);								  // 摩擦轮斜坡
		Slope_NextVal(&shooter.fricSlope);									  // 斜坡下一个值
		shooter.fricMotor[0].targetSpeed = -Slope_GetVal(&shooter.fricSlope); // 摩擦轮速度
		shooter.fricMotor[1].targetSpeed = Slope_GetVal(&shooter.fricSlope);
	}
}


uint8_t shootflag = 0;
// 摇杆控制
void Shooter_RockerCtrl()
{	
	static int lastwheel = 0;
	if(chassis.pattern==Chassis_control)
	{
		if(rcInfo.wheel>400&&lastwheel<=400)
		{
			if(shooter.fricOpenFlag)
			{
				shooter.fricOpenFlag=0;
				shooter.fricMotor[0].targetSpeed = 0 ;
				shooter.fricMotor[1].targetSpeed =0;
			}
			else
			{
				shooter.fricMotor[0].targetSpeed = -shooter.fricSpd ;
				shooter.fricMotor[1].targetSpeed = shooter.fricSpd ;
				shooter.fricOpenFlag=1;
			
			}
		}
		lastwheel = rcInfo.wheel;
		
		if(!gimbal.visionEnable)
		{
			if(rcInfo.left==1&&shooter.fricOpenFlag)
			{
				shooter.workState=TRIGGER_CONTINUE;
			}
			else	
				shooter.workState=IDLE;
		}
	}
}

void Task_Shooter_Callback()
{	
	static uint8_t over_speed, down_speed=0;
	// heat=USER_JudgeData.shooter_barrel_heat_limit;
	if(USER_JudgeData.initial_speed >10)
	shooter.bullet_speed = USER_JudgeData.initial_speed;
	
	if(shooter.bullet_speed!=shooter.last_bullet_speed)
	{
		//22
		if(shooter.bullet_speed>22.5f)//23
		{
			over_speed++;
			if(over_speed>=2)
			{
				over_speed=0;
				shooter.fricSpd-=60;
				shooter.fricMotor[0].targetSpeed+= 60;
				shooter.fricMotor[1].targetSpeed-= 60;
				down_speed=0;
			}
		}
		else if(shooter.bullet_speed<21.5f)//21
		{
			down_speed++;
			if(down_speed>=4)
			{
				down_speed=0;
				shooter.fricSpd+=50;
				shooter.fricMotor[0].targetSpeed-= 50;
				shooter.fricMotor[1].targetSpeed+= 50;
			}
		}
		
	}
	
	shooter.last_bullet_speed = shooter.bullet_speed; 
	
	if(Rocker_Ctrl)
		Shooter_RockerCtrl();
	Shooter_state(shooter.fricOpenFlag);

	
//堵转处理
//电机角度与目标角度相差超过10度则进行堵转判定
	if(ABS(shooter.triggerMotor.totalAngle-shooter.triggerMotor.targetAngle)>MOTOR_M2006_DGR2CODE(10)&&shooter.workState!=TRIGGER_REVERSE) 
	{
		shooter.block.judgeCnt++; //堵转判定计数器++
		if(shooter.block.judgeCnt>100) //计数器达到一定值，则判定为堵转，触发反转
		{
			shooter.block.judgeCnt=0;
			shooter.block.reverseCnt++;
			shooter.workState=TRIGGER_REVERSE;
			if (shooter.block.reverseCnt > 5)
			{
				shooter.block.state = 1;
			}
			else
				shooter.block.state = 0;
		}
	}
	else //与目标值相差小于10度，拨弹状态正常，将堵转判定计数器归零
	{
		shooter.block.judgeCnt=0;
	}
  Heat_Limit();
    //拨弹
  switch(shooter.workState)
  {
		case TRIGGER:
		{
			if(JUDGE_IsValid()==false||Heat_Limit())   //未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
			{
				if(shooter.triggerMotor.targetAngle-shooter.triggerMotor.totalAngle<MOTOR_M2006_DGR2CODE(8))
				{
					shooter.triggerMotor.targetAngle+=MOTOR_M2006_DGR2CODE(360*1/8.0*1);  //每次转动1/8圈
					shooter.workState=IDLE;    
					shooter.number +=1;       
					osDelay(t);
				}
			}
			else
			{
				shooter.workState=IDLE;
			}
		}
		break;

		case TRIGGER_CLICK:
		{
			if(JUDGE_IsValid()==false||Heat_Limit())   //未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
			{ 
				if(shooter.triggerMotor.targetAngle-shooter.triggerMotor.totalAngle<MOTOR_M2006_DGR2CODE(8))
				{
					shooter.triggerMotor.targetAngle+=MOTOR_M2006_DGR2CODE(360*1/8.0*1);  //每次转动1/8圈
					shooter.workState=IDLE;    
					shooter.number +=1;  
					osDelay(t);						
				}			 			
			}
		else
			{
				shooter.workState=IDLE;
			}
		}
		break;

		case TRIGGER_CONTINUE:		
			if(JUDGE_IsValid()==false||Heat_Limit())   //未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
			{ 
				if(shooter.triggerMotor.targetAngle-shooter.triggerMotor.totalAngle<MOTOR_M2006_DGR2CODE(8))
				{
					shooter.triggerMotor.targetAngle+=MOTOR_M2006_DGR2CODE(360*1/8.0*1);  //每次转动1/8圈    
					shooter.number +=1;    
					osDelay(t);						
				}						
			}
	    else
			{
					shooter.workState=IDLE;
			}
		break;

		case TRIGGER_REVERSE: 
					 //嘀嘀嘀
			Beep_PlayNotes((Note[]){{T_M1,D_Sixteenth},{T_M1,D_Sixteenth},{T_M1,D_Sixteenth}},3);          
			shooter.triggerMotor.targetAngle-=MOTOR_M2006_DGR2CODE(360*1.5/8.0*1); //电机反向拨动1.5/8圈    
			osDelay(500);
			shooter.triggerMotor.targetAngle+=MOTOR_M2006_DGR2CODE(360*0.5/8.0*1); //正转0.5/8圈                 
			shooter.workState=IDLE;
		break;    
		
		default:
		break;
	}
}


void OS_ShooterCallback(void const * argument)
{
	osDelay(100);
	Shooter_Init();
	PID_Clear(&shooter.triggerMotor.anglePID.inner);
	PID_Clear(&shooter.triggerMotor.anglePID.outer);
	osDelay(100);
	for(;;)
	{
		if(true /* JUDGE_GetShooterOutputState()||1 */)
		{
			Task_Shooter_Callback();
		}
		osDelay(10);
	}
}


