#include "Chassis.h"
#include "USER_Moto.h"
#include "bsp_can.h"
#include "USER_RC.h"
#include "user_lib.h"
#include "arm_math.h"
#include "Gimbal.h"
#include "vision.h"


int8_t diagonal_enable = 0;
float vx,vy,vw = 0;  //AI传来的旋转速度
float wheelvx[4];
float wheelvy[4];
int32_t wheelRPM[4];
float targetangle[4];

Chassis_t chassis = {0};

float vx_test;
float vy_test;
float mode_test;

/********************初始化************************/
void Chassis_Init()
{
	// 底盘尺寸信息（用于解算轮速）
	chassis.info.wheelbase = 320;
	chassis.info.wheeltrack = 320;
	chassis.info.wheelRadius = 115;
	chassis.info.offsetX = 0; // 15
	chassis.info.offsetY = 0; //-10
	chassis.info.R = sqrtf(powf(chassis.info.wheelbase / 2 + chassis.info.offsetX, 2) + powf(chassis.info.wheeltrack / 2 + chassis.info.offsetY, 2));
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

	Chassis_InitPID();
}

void Chassis_InitPID()
{
	PID_Init(&chassis.rotate.pid, 0.2, 0, 0.3, 2, 15); // 15	PID_Init(&chassis.rotate.pid, 0.4, 0.001, 0.15, 0, 15); // 15
	PID_SetDeadzone(&chassis.rotate.pid, 0.1);
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
            chassis.pattern = Chassis_AI;
            break;

        case 1:
            chassis.pattern = Chassis_control;
            break;

        default:
            break;
    }
	//不同控制模式下 不同状态机
    switch(chassis.pattern)
    {
        case Chassis_control: //人控模式
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
            break;

        case Chassis_AI: //AI模式
            if(vision_receive.spin_mode)
            {
				chassis.rotate.mode = ChassisMode_Spin;
            }
            else
            {
                chassis.rotate.mode = ChassisMode_Follow;
            }
            break;

        default:
            break;
    }
}

/*更新移动数据*/
void Chassis_UpdateMove(void)
{
	float gimbalAngleSin=sin(chassis.rotate.relativeAngle*PI/180);
	float gimbalAngleCos=cos(chassis.rotate.relativeAngle*PI/180);

    if(chassis.pattern == Chassis_AI) //ai模式
    {
        Slope_SetTarget(&chassis.move.xSlope, 1.5f * vx);
        Slope_SetTarget(&chassis.move.ySlope, 1.5f * vy);
    }
    else
    {
        Slope_SetTarget(&chassis.move.xSlope,(float)rcInfo.ch3 * chassis.move.maxVx / 660);
        Slope_SetTarget(&chassis.move.ySlope,(float)rcInfo.ch4 * chassis.move.maxVy / 660);
    }
	chassis.move.vx=-(Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin);
	chassis.move.vy=(-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos);
    Chassis_UpdateSlope();
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
    PID_SingleCalc(&chassis.rotate.pid, 0, -angle);
    chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;
    LIMIT(chassis.move.vw,-chassis.move.maxVw,chassis.move.maxVw);
    if(gimbal.visionEnable == true)
    {
        chassis.move.vw = 0;
    }
	if (chassis.pattern == Chassis_AI)
	{
		chassis.move.vw = vw;
	}
}

static void Chassis_HandleSpin(void) //小陀螺模式
{
    float ratio;
    if(chassis.pattern == Chassis_control)
    {
        if(ABS(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + ABS(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy > 0.05f)
            ratio = 0.6f;
        else
            ratio = 1.0f;
    }
    else
    {
        ratio = 0.5f;
    }
    Slope_SetTarget(&chassis.move.spinSlope,chassis.move.maxVw * ratio);
	chassis.move.vw =chassis.move.spinSlope.value;
}


/*更新旋转斜坡和移动斜坡 工具*/
void Spin_SpeedUpdate() //
{
	chassis.move.maxVw = 15.0f; // chassis.move.maxPower * 0.225f + 3.25f
	if (chassis.move.maxVw <= 0.5f)
	{
		chassis.move.maxVw = 0.5f;
	}

	if (chassis.rotate.mode == ChassisMode_Spin)
	{
		chassis.move.maxVx = 2159.6f;//0.4985f * chassis.move.maxPower * chassis.move.maxPower - 40.994f * chassis.move.maxPower + 2159.6f
		if (chassis.move.maxVx <= 0)
			chassis.move.maxVx = 0;
		chassis.move.maxVy = chassis.move.maxVx;
	}
	else
	{
		chassis.move.maxVx = 5500;
		chassis.move.maxVy = 5500;
	}
}

void Chassis_UpdateSlope()
{
	Slope_NextVal(&chassis.move.xSlope);
	Slope_NextVal(&chassis.move.ySlope);
	Slope_NextVal(&chassis.move.spinSlope);
	Spin_SpeedUpdate();
	float rotateRatio = (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f;
	chassis.move.maxVx = WHEELSPEED_MAX / 60.0f / (268.f / 17.f) * 2 * PI * chassis.info.wheelRadius;
	chassis.move.maxVy = chassis.move.maxVx;
	chassis.move.maxVw = WHEELSPEED_MAX / rotateRatio / 60.0f / (268.f / 17.f) * 2.0f * PI * chassis.info.wheelRadius * 1.0f / 1.414f;
    chassis.move.maxVw = 15.0f;
}


/************************freertos任务**********************
以下任务受freertos操作系统调度
**********************************************************/

// 底盘任务回调函数
void Task_Chassis_Callback()
{
	Chassis_UpdateAngle(); //更新底盘云台之间关联角
	Chassis_ModeCtrl();	//更新模式状态机
	Chassis_UpdateMove(); //更新底盘移动数据
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
	// chassis.move.vw = (float)rcInfo.ch1*chassis.move.maxVw/-660;//暂时用拨杆控制旋转速度
	/***全向轮解算各轮子转速****/
	//计算旋转半径和45度角的三角函数值
    // R = 质心到轮中心的距离
    float cos45 = 0.707106f; // 即 1 / 1.414f
    
    float wheel_v[4];
    wheel_v[0] = (chassis.move.vx + chassis.move.vy) * cos45 + chassis.move.vw * chassis.info.R;    //左前
    wheel_v[1] = (chassis.move.vx - chassis.move.vy) * cos45 + chassis.move.vw * chassis.info.R;    //右前
    wheel_v[2] = (-chassis.move.vx + chassis.move.vy) * cos45 + chassis.move.vw * chassis.info.R;    //左后
    wheel_v[3] = (-chassis.move.vx - chassis.move.vy) * cos45 + chassis.move.vw * chassis.info.R;    //右后
    //转换为电机 RPM
    float rpm_ratio = 60.0f / (2.0f * PI * chassis.info.wheelRadius) * (268.0f / 17.0f);
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis.motors[i].targetSpeed = wheel_v[i] * rpm_ratio;
        // chassis.motors[i].targetSpeed = 0;//测试用
    }
}


void OS_ChassisCallback(void const * argument)
{
	osDelay(500);
	Chassis_Init();
    for(;;)
    {
		Task_Chassis_Callback();
        osDelay(2);
    }
}
