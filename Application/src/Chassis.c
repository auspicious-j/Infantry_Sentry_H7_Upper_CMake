#include "Chassis.h"
#include "PID.h"
#include "USER_Moto.h"
#include "USER_RC.h"
#include "bsp_can.h"
#include "pid.h"
#include "user_lib.h"
#include "arm_math.h"
#include "Gimbal.h"
#include "vision.h"
#include <stdbool.h>
#include <stdint.h>


int8_t diagonal_enable = 0;
float vx,vy,vw = 0;  //AI传来的旋转速度
float wheelvx[4];
float wheelvy[4];
int32_t wheelRPM[4];
uint16_t SET_WHEELSPEED_MAX = 8000;
float targetangle[4];

Chassis_t chassis = {0};

float vx_test;
float vy_test;
float mode_test;

/********************初始化************************/
void Chassis_Init()
{
	// 底盘尺寸信息（用于解算轮速）
	chassis.info.wheelbase = 420;
	chassis.info.wheeltrack = 420;
	chassis.info.wheelRadius = 75;
	chassis.info.offsetX = 0; // 15
	chassis.info.offsetY = 0; //-10
	chassis.info.R = sqrtf(powf(chassis.info.wheelbase / 2 + chassis.info.offsetX, 2) + powf(chassis.info.wheeltrack / 2 + chassis.info.offsetY, 2));
	chassis.info.rpm_ratio = 60.0f / (2.0f * PI * chassis.info.wheelRadius) * (268.0f / 17.0f);

    // 移动参数初始化

	// 旋转参数初始化
	chassis.rotate.InitAngle = INIT_YAW_ANGLE; 
	if (chassis.rotate.InitAngle > 360)
		chassis.rotate.InitAngle -= 360;
	chassis.rotate.InitpitchAngle = 1290; 

	// 斜坡函数初始化
	Slope_Init(&chassis.move.xSlope, 40, 0);
	Slope_Init(&chassis.move.ySlope, 40, 0);
	Slope_Init(&chassis.move.spinSlope, 0.1, 0);
	Slope_Init(&chassis.move.outputSlope, 0.1, 0);
	Slope_Init(&chassis.move.chargeSlope, 0.15, 0);

    Filter_InitAverFilter(&gimbal.Mouse.yawFilter, 10); //
    Filter_InitAverFilter(&gimbal.Mouse.pitchFilter, 2);
    Filter_InitAverFilter(&gimbal.visionFilter.find, 25); // 

	Chassis_InitPID();
    Chassis_RegisterEvents();
}

void Chassis_InitPID()
{
	PID_Init(&chassis.rotate.pid, 0.2, 0, 6, 4, 15); 
    // PID_SetDeadzone(&chassis.rotate.pid, 0.1);变成手动死区
    // PID_Init(&chassis.move.real_xPID, 1, 0, 0, 0, 2000); // 15
    // PID_Init(&chassis.move.real_yPID, 1, 0, 0, 0, 2000); // 15
    PID_Init(&chassis.move.real_wPID, 0, 0, 0, 0, 10); // 15
}
/**底盘云台关联角度更新**/
void Chassis_UpdateAngle(void)
{
    chassis.rotate.InitAngle = INIT_YAW_ANGLE;

    if(chassis.rotate.InitAngle >= 360)
        chassis.rotate.InitAngle -= 360;

    if(chassis.rotate.InitAngle < 0)
        chassis.rotate.InitAngle += 360;

    uint16_t yaw_online = 0; //判断云台电机是否离线
    if(yaw_online == 0)
    {
        chassis.rotate.nowAngle = gimbal.base_yawMotor.nowAngle;
        chassis.rotate.relativeAngle = chassis.rotate.nowAngle - chassis.rotate.InitAngle;
    }
    else
    {
        chassis.rotate.relativeAngle = 0;
    }
}

/*******模式状态机切换*********/

void Chassis_ModeCtrl(void) 
{
	//决定ai控还是人控
    switch(rcInfo.right)
    {
        case 3:
            chassis.pattern = Chassis_control;
            break;

        default:
            break;
    }
	//不同控制模式下 不同状态机
    switch(chassis.pattern)
    {
        case Chassis_control: //人控模式
            if(Rocker_Ctrl == true){
                switch(chassis.rotate.mode)
                {
                    case ChassisMode_Follow:
                        if(rcInfo.left == 2)
                        {
                            chassis.rotate.mode = ChassisMode_Spin;
                        }
                        break;

                    case ChassisMode_Spin:

                        if(rcInfo.left == 3)
                        {
                            chassis.rotate.mode = ChassisMode_Follow;
                        }
                        break;

                    default:
                        break;
                }
            }
            break;
        default:
            break;
    }
}
/*************************RC事件**************************
以下任务受键鼠event调度
*********************************************************/
void Chassis_RegisterEvents()
{
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnDown, Chassis_Move_KeyCallback); 
	RC_Register(Key_W | Key_A | Key_S | Key_D, CombineKey_None, KeyEvent_OnUp, Chassis_Stop_KeyCallback);	
	RC_Register(Key_Q | Key_E | Key_G, CombineKey_None, KeyEvent_OnDown, Chassis_SwitchMode_KeyCallback);	
	RC_Register(Key_V, CombineKey_None, KeyEvent_OnDown, Chassis_capOutputChange_KeyCallback);
	// RC_Register(Key_E | Key_R, CombineKey_Ctrl, KeyEvent_OnDown, Chassis_Return_KeyCallback);

	//  RC_Register(Key_V,CombineKey_Shift,KeyEvent_OnDown,Chassis_capBurstChange_KeyCallback);
	//	RC_Register(Key_Shift,CombineKey_None,KeyEvent_OnDown,Cap_On_KeyCallback);
	RC_Register(Key_Shift, CombineKey_None, KeyEvent_OnUp, UI_UPdate);
	RC_Register(Key_Shift, CombineKey_None, KeyEvent_OnDown, UI_UPdate);
}

void Chassis_Move_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_W:
		chassis.key.key_w = -1;
		break;
	case Key_S:
		chassis.key.key_s = 1;
		break;
	case Key_D:
		chassis.key.key_d = 1;
		break;
	case Key_A:
		chassis.key.key_a = -1;
		break;
	default:
		break;
	}
}

void UI_UPdate(KeyType key, KeyCombineType combine, KeyEventType event)
{
	// if (event == KeyEvent_OnDown)
    	// UIupdateState = 1;
	// else
		// UIupdateState = 0;
}

// 停止移动按键回调
void Chassis_Stop_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	switch (key)
	{
	case Key_W:
		chassis.key.key_w = 0;
		break;
	case Key_S:
		chassis.key.key_s = 0;
		break;
	case Key_D:
		chassis.key.key_d = 0;
		break;
	case Key_A:
		chassis.key.key_a = 0;
		break;
	default:
		break;
	}
}

void Chassis_capOutputChange_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (chassis.move.fastMode == 0)
	{
		chassis.move.fastMode = 1;
		return;
	}
	if (chassis.move.fastMode == 1)
		chassis.move.fastMode = 0;
}

void Chassis_SwitchMode_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	if (chassis.rotate.mode != ChassisMode_Follow) //  Q/E/R???g? ????????????g?
	{
		chassis.rotate.mode = ChassisMode_Follow;
	}
	else
	{
		switch (key)
		{
		case Key_Q: // 小陀螺模式
			PID_Clear(&chassis.rotate.pid);
			chassis.rotate.mode = ChassisMode_Spin;
			Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw);
			break;
    case Key_E://关底盘跟随
      if (chassis.rotate.pid.maxOutput == 0)
      {
        chassis.rotate.pid.maxOutput = 15;
      }
      else
      {
        chassis.rotate.pid.maxOutput = 0;
      }
		break;
		default:
			break;
		}
	}
}


/*更新移动数据*/
void Chassis_UpdateMove(void)
{
	float gimbalAngleSin=sin(chassis.rotate.relativeAngle*PI/180);
	float gimbalAngleCos=cos(chassis.rotate.relativeAngle*PI/180);
    Chassis_UpdateSlope();
    if(chassis.rotate.mode == ChassisMode_Spin)
    {
        chassis.move.maxVx *= (chassis.rotate.ratio);
        chassis.move.maxVy *= (chassis.rotate.ratio);
    }

    if (Rocker_Ctrl)
    {
        Slope_SetTarget(&chassis.move.xSlope,(float)rcInfo.ch3 * chassis.move.maxVx / 660);
        Slope_SetTarget(&chassis.move.ySlope,(float)rcInfo.ch4 * chassis.move.maxVy / 660);
    }
    else 
    {
        Slope_SetTarget(&chassis.move.ySlope, chassis.move.maxVy * (chassis.key.key_w + chassis.key.key_s)); // ?????????
        Slope_SetTarget(&chassis.move.xSlope, chassis.move.maxVx * (chassis.key.key_d + chassis.key.key_a)); // ?????????
    }

	chassis.move.vx=-(Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin);
	chassis.move.vy=(-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos);
}

/*旋转状态机*/
static void Chassis_HandleFollow(void) //底盘跟随模式
{
    Slope_SetTarget(&chassis.move.spinSlope, 0);
    float angle;
    if(chassis.pattern == Chassis_control)
    {
        angle = chassis.rotate.relativeAngle;
    }
    if(angle >= 180)
        angle -= 360;
    if(angle < -180)
        angle += 360;
    float deadzone = 0.1f;
    float pid_angle = 0.0f;
    if (angle > deadzone)
    {
        pid_angle = angle - deadzone;
    }
    else if (angle < -deadzone)
    {
        pid_angle = angle + deadzone;
    }
    else
    {
        pid_angle = 0.0f;
        chassis.rotate.pid.integral = 0.0f;
    }
    PID_SingleCalc(&chassis.rotate.pid, 0, -pid_angle);
    chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;
    LIMIT(chassis.move.vw,-chassis.move.maxVw,chassis.move.maxVw);
}

static void Chassis_HandleSpin(void) //小陀螺模式
{
    if(chassis.pattern == Chassis_control)
    {
        if(ABS(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + ABS(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy > 0.05f)
            chassis.rotate.ratio = 0.4f;
        else
            chassis.rotate.ratio = 1.0f;
    }
    else
    {
        chassis.rotate.ratio = 0.5f;
    }
    Slope_SetTarget(&chassis.move.spinSlope,chassis.move.maxVw * chassis.rotate.ratio);
	chassis.move.vw =chassis.move.spinSlope.value;
}

void Chassis_UpdateSlope()
{
	Slope_NextVal(&chassis.move.xSlope);
	Slope_NextVal(&chassis.move.ySlope);
	Slope_NextVal(&chassis.move.spinSlope);
	float rotateRatio = (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f;
	chassis.move.maxVx = SET_WHEELSPEED_MAX / 60.0f / 14.88 * 2 * PI * chassis.info.wheelRadius;
	chassis.move.maxVy = chassis.move.maxVx;//60是rpm转换成转速 14.88是减速箱减速比 2PI*R是轮子周长
	chassis.move.maxVw = SET_WHEELSPEED_MAX / rotateRatio / 60.0f / 14.88 * 2.0f * PI * chassis.info.wheelRadius * 1.0f / 1.414f;
}


/************************freertos任务**********************
以下任务受freertos操作系统调度
**********************************************************/

// 底盘任务回调函数
void Task_Chassis_Callback()
{
    // if (rcInfo.right == 1) 
    // {
    //     chassis.move.fastMode = 1;
    // }
    // else 
    // {
    //     chassis.move.fastMode = 0;
    // }
    Chassis_UpdateAngle(); //更新底盘云台之间关联角
	Chassis_ModeCtrl();	//更新模式状态机
    switch(chassis.rotate.mode) //更新两种旋转模式状态机
    {
        case ChassisMode_Follow:
            Chassis_HandleFollow();
            break;
        case ChassisMode_Spin:
            Chassis_HandleSpin();
            break;
        default:
            break;
	}
	Chassis_UpdateMove(); //更新底盘移动数据

	/***全向轮解算各轮子转速****/
	//计算旋转半径和45度角的三角函数值
    // R = 质心到轮中心的距离
    float cos45 = 0.707106f; // 即 1 / 1.414f
    //先反解车当前真实速度
    float real_wheel_v[4];
    for (uint8_t i = 0; i < 4; i++) 
    {
        real_wheel_v[i] = chassis.motors[i].speed / chassis.info.rpm_ratio;
    }
    chassis.move.real_vx = (real_wheel_v[0] + real_wheel_v[1] - real_wheel_v[2] - real_wheel_v[3]) / (4.0f * cos45);
    chassis.move.real_vy = (real_wheel_v[0] - real_wheel_v[1] + real_wheel_v[2] - real_wheel_v[3]) / (4.0f * cos45);
    chassis.move.real_vw = (real_wheel_v[0] + real_wheel_v[1] + real_wheel_v[2] + real_wheel_v[3]) / (4.0f * chassis.info.R);

    PID_SingleCalc(&chassis.move.real_xPID, chassis.move.vx, chassis.move.real_vx);//PID进行修正
    PID_SingleCalc(&chassis.move.real_yPID, chassis.move.vy, chassis.move.real_vy);
    PID_SingleCalc(&chassis.move.real_wPID, chassis.move.vw, chassis.move.real_vw);
    
    static float ctrl_vx, ctrl_vy, ctrl_vw;
    ctrl_vx = chassis.move.vx + chassis.move.real_xPID.output;
    ctrl_vy = chassis.move.vy + chassis.move.real_yPID.output;
    ctrl_vw = chassis.move.vw + chassis.move.real_wPID.output;

    float wheel_v[4];
    wheel_v[0] = (ctrl_vx + ctrl_vy) * cos45 + ctrl_vw * chassis.info.R;    //左前
    wheel_v[1] = (ctrl_vx - ctrl_vy) * cos45 + ctrl_vw * chassis.info.R;    //右前
    wheel_v[2] = (-ctrl_vx + ctrl_vy) * cos45 + ctrl_vw * chassis.info.R;    //左后
    wheel_v[3] = (-ctrl_vx - ctrl_vy) * cos45 + ctrl_vw * chassis.info.R;    //右后
    //转换为电机 RPM
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis.motors[i].targetSpeed = wheel_v[i] * chassis.info.rpm_ratio;
    }
}


void OS_ChassisCallback(void const * argument)
{
	osDelay(1500);
	Chassis_Init();
    for(;;)
    {
		Task_Chassis_Callback();
        osDelay(2);
    }
}
