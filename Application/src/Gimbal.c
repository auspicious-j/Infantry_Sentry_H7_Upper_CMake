#include "Gimbal.h"
#include "PID.h"
#include "gimbal.h"
#include "imu_temp_ctrl.h"
#include "USER_RC.h"
#include "vision.h"
#include "chassis.h"
#include "shooter.h"
#include "arm_math.h"

#define DEG_TO_RAD 0.01745329f  // PI / 180
#define TOP_YAW_LIMIT 45.0f
#define BASE_YAW_J 0.04575f


Gimbal_t gimbal;

void Gimbal_InitPID(void);

/********************初始化************************/
// 初始化云台
void Gimbal_Init()
{
	gimbal.top_pitch.pitchMax = 60; // 设定pitch角度限幅 (绝对角度) (目前没有用到 小pitch限位由大小pitch的相对角度来决定)
	gimbal.top_pitch.pitchMin = 10;

	gimbal.top_pitch.relativePitchMax = -30; // 相对角度限幅 (相对于fold_pitch)
	gimbal.top_pitch.relativePitchMin = -100;// 在gimbal_updateangle函数中根据fold_pitch的角度动态更新相对角度限幅

	gimbal.top_pitch.initAngle = 0; // 陀螺仪pitch开机角度 *0表示pitch水平
	gimbal.top_pitch.targetAngle = gimbal.top_pitch.initAngle;

	gimbal.fold_pitch.pitchMax = 90;
	gimbal.fold_pitch.pitchMin = 5;

	gimbal.fold_pitch.initAngle = 90;
	gimbal.fold_pitch.targetAngle = gimbal.fold_pitch.initAngle;

	gimbal.base_yaw.initAngle = 0; // 陀螺仪yaw开机角度   *0表示yaw不动
	gimbal.base_yaw.targetAngle = gimbal.base_yaw.initAngle;

	gimbal.top_yaw.initAngle = 0;
	gimbal.top_yaw.targetAngle = gimbal.top_yaw.initAngle;


	Filter_InitAverFilter(&gimbal.visionFilter.find,50);
	gimbal.visionEnable = false;
	
	gimbal.scan.yaw_speed = 60.0f;
	gimbal.scan.pitch_phase = 0.0f;
	gimbal.scan.pitch_freq = 1.0f;
	gimbal.scan.pitch_amp = 10.0f;
	gimbal.scan.pitch_offset = -5.0f;
	gimbal.scan.last_ideal_pitch = 0.0f;
	gimbal.scan.init_flag = 1;
	Gimbal_InitPID();         // 初始化PID参数
}

void Gimbal_InitPID()
{
	/*pitch由陀螺仪控制*/
	PID_Init(&gimbal.top_pitch.imuPID.inner, 6, 0, 0.4, 500, 7000);
	DEPID_Init(&gimbal.top_pitch.imuPID.deOuter, 75, 0.7, 120, 200, 7000, 0.6);//45, 0.7, 50串级pid
	PID_Init(&gimbal.top_pitch.imuPID.outer,-0.5,-0.001,-5,1,10); //自己写位置环 mit速度环

	PID_Init(&gimbal.fold_pitch.imuPID.outer, 1, 0.001, 10, 3, 15);

	PID_Init(&gimbal.base_yaw.imuPID.inner,5.8,0.02,3.5,1000,7000);
	DEPID_Init(&gimbal.base_yaw.imuPID.deOuter,37,0.03,173,700,2000,0.45);	//串级pid
	PID_Init(&gimbal.base_yaw.imuPID.outer,300,0.5,5000,1000,50000);//自己写位置环 mit速度环

	PID_Init(&gimbal.top_yaw.imuPID.inner, 50, 0.1, 10, 7000, 30000);//270 0.3  300
	DEPID_Init(&gimbal.top_yaw.imuPID.deOuter, 100, 0.4, 150, 100, 30000, 0.7);
}

/*云台角度更新*/
void Gimbal_UpdateAngle()
{
	gimbal.top_yaw.gyro = -INS.gyro[1];
	gimbal.top_yaw.angle = INS.yaw;
	gimbal.base_yaw.angle = gimbal.top_yaw.angle - (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0 * 360.0f; //得出大yaw的imu角度
	gimbal.base_yaw.gyro = gimbal.base_yawMotor.para.vel;
	gimbal.top_pitch.gyro = -INS.gyro[0];
	gimbal.top_pitch.angle = INS.pitch;
	gimbal.fold_pitch.angle = gimbal.fold_pitchMotor.nowAngle - FOLD_PITCH_OFFSET / PI * 180.0f;//根据折叠pitch的电机编码器值控制
	gimbal.fold_pitch.IMU_angle = gimbal.top_pitch.angle + (gimbal.top_pitchMotor.para.pos - TOP_PITCH_OFFSET) / PI * 180.0f + 90.0f;//根据顶上pitch的imu角度和电机编码器值得出折叠pitch的imu角度
	gimbal.fold_pitch.gyro = gimbal.fold_pitchMotor.para.vel;
	
	//pitch角度归一化到-180~180度
	while (gimbal.fold_pitch.angle > 180.0f)
		gimbal.fold_pitch.angle -= 360.0f;
	while (gimbal.fold_pitch.angle < -180.0f)
		gimbal.fold_pitch.angle += 360.0f;

	//更新top_pitch限幅 根据fold_pitch角度动态限幅
	if (gimbal.fold_pitch.angle >= 70.0f && gimbal.fold_pitch.angle <= 95.0f)
	{
		// fold_pitch角度在95-70度之间时，相对角度限幅为 -20到-110度
		gimbal.top_pitch.relativePitchMax = -30.0f;
		gimbal.top_pitch.relativePitchMin = -110.0f;
	}
	else if (gimbal.fold_pitch.angle >= 0.0f && gimbal.fold_pitch.angle < 70.0f)
	{
		// fold_pitch角度在70-0度之间时，映射相对角度限幅从0到-110度映射到从0到0度
		// 线性插值：relativePitchMin = -1.5 * fold_pitch_angle
		gimbal.top_pitch.relativePitchMax = 0.0f;
		gimbal.top_pitch.relativePitchMin = -1.5f * gimbal.fold_pitch.angle;
	}
	else
	{
		// 其他角度范围，使用默认值
		gimbal.top_pitch.relativePitchMax = -30.0f;
		gimbal.top_pitch.relativePitchMin = -5.0f;
	}

	// yaw角度累计
	float dAngle = 0;
	if (gimbal.top_yaw.angle - gimbal.top_yaw.lastAngle < -270)
		dAngle = gimbal.top_yaw.angle + (360 - gimbal.top_yaw.lastAngle);
	else if (gimbal.top_yaw.angle - gimbal.top_yaw.lastAngle > 270)
		dAngle = -gimbal.top_yaw.lastAngle - (360 - gimbal.top_yaw.angle);
	else
		dAngle = gimbal.top_yaw.angle - gimbal.top_yaw.lastAngle;
	gimbal.top_yaw.totalAngle += dAngle;

	// target += round((total - target) / 360.0) * 360.0;
	while (gimbal.top_yaw.targetAngle - gimbal.top_yaw.totalAngle >= 180 || gimbal.top_yaw.totalAngle - gimbal.top_yaw.targetAngle >= 180)
	{
		if (gimbal.top_yaw.targetAngle - gimbal.top_yaw.totalAngle >= 180)
		{
			gimbal.top_yaw.targetAngle = gimbal.top_yaw.targetAngle - 360;
		}
		if (gimbal.top_yaw.totalAngle - gimbal.top_yaw.targetAngle >= 180)
		{
			gimbal.top_yaw.targetAngle = gimbal.top_yaw.targetAngle + 360;
		}
	}
	gimbal.top_yaw.lastAngle = gimbal.top_yaw.angle;
	gimbal.base_yaw.totalAngle = gimbal.top_yaw.totalAngle - (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0 * 360.0;
}

/*云台PID参数更新*/
static Chassis_Mode_e last_mode;
void Gimbal_UpdatePID(void)
{
    if(last_mode == chassis.rotate.mode)
    {
        return;
    }
    last_mode = chassis.rotate.mode;

    switch(chassis.rotate.mode)
    {
        case ChassisMode_Spin:
            // gimbal.base_yaw.imuPID.outer.maxIntegral = 6000;
            // gimbal.base_yaw.imuPID.outer.ki = 0.5f;
            break;

        default:
            // gimbal.base_yaw.imuPID.outer.maxIntegral = 1000;
            // gimbal.base_yaw.imuPID.outer.ki = 0.1f;
            break;
    }
}

/****模式更改*****/
void Gimbal_ModeCtrl(void)
{
	if(chassis.pattern == Chassis_AI)
    {
        gimbal.ctrl_mode = GimbalCtrl_AI;
    }
    else
    {
        gimbal.ctrl_mode = GimbalCtrl_Control;
    }
    switch(gimbal.ctrl_mode)
    {
        case GimbalCtrl_Control: //人控模式
            if(rcInfo.wheel < -400)
            {
                gimbal.state = GimbalState_Vision;
            }
            else
            {
                gimbal.state = GimbalState_Rocker;
            }
            break;

        case GimbalCtrl_AI: 	//AI模式
            if(rcInfo.left == 1)
            {
                gimbal.state = GimbalState_Scan;
            }
			else if (rcInfo.left == 2) 
			{
				gimbal.state = GimbalState_Fold;
			}
            else
            {
                gimbal.state = GimbalState_Rocker;
            }
            break;

        default:
            break;
    }
}


static void Gimbal_HandleRocker(void)
{
    gimbal.visionEnable = false;
	Gimbal_RockerCtrl();
}

static void Gimbal_HandleFold(void)
{
	gimbal.visionEnable = false;
	Gimbal_FoldCtrl();
}

static void Gimbal_HandleVision(void)
{
	gimbal.visionEnable = true;
	// shooter.fricOpenFlag = 0;
	if(vision.found)
		Gimbal_VisionCtrl();
	else
		Gimbal_RockerCtrl();
}

static void Gimbal_HandleScan(void)
{
    gimbal.visionEnable = true;
    if(vision.found)
    {
        shooter.fricOpenFlag = 1;
        Shooter_state(shooter.fricOpenFlag);
        Gimbal_VisionCtrl();
    }
    else
    {
        shooter.fricOpenFlag = 0;
        Shooter_state(shooter.fricOpenFlag);
        Gimbal_ScanCtrl();
    }
}



/*******三种控制函数*********/
void Gimbal_RockerCtrl()
{
	gimbal.top_yaw.targetAngle -= rcInfo.ch1 * 0.3 / 660.0f;	// yaw
	gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle;
	gimbal.top_pitch.targetAngle += rcInfo.ch2 * 0.35 / 660.0f; // 旋转云台pitch
	// 小pitch限位：相对于fold_pitch的相对角度限制
	float relativeAngle = gimbal.top_pitch.targetAngle - gimbal.fold_pitch.IMU_angle;
	LIMIT(relativeAngle, gimbal.top_pitch.relativePitchMin, gimbal.top_pitch.relativePitchMax);
	gimbal.top_pitch.targetAngle = gimbal.fold_pitch.IMU_angle + relativeAngle;	// 大pitch限位：绝对角度限制
}

void Gimbal_FoldCtrl()
{
	gimbal.top_yaw.targetAngle -= rcInfo.ch1 * 0.3 / 660.0f;	// yaw
	gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle;
	gimbal.fold_pitch.targetAngle += rcInfo.ch2 * 0.2 / 660.0f; // 旋转云台pitch
	LIMIT(gimbal.fold_pitch.targetAngle, gimbal.fold_pitch.pitchMin, gimbal.fold_pitch.pitchMax);//折叠pitch限位
	// 小pitch限位：根据当前fold_pitch角度动态调整top_pitch的相对角度限幅
	float relativeAngle = gimbal.top_pitch.targetAngle - gimbal.fold_pitch.IMU_angle;
	LIMIT(relativeAngle, gimbal.top_pitch.relativePitchMin, gimbal.top_pitch.relativePitchMax);
	gimbal.top_pitch.targetAngle = gimbal.fold_pitch.IMU_angle + relativeAngle;  //翻译一下就是限幅过的relative angle
}

void Gimbal_VisionCtrl()
{
    float target_top  = vision.top_yaw;
    float target_base = vision.base_yaw;//目标位置
    int base_cycle;
    if((gimbal.base_yaw.targetAngle / 360.f) > 0)
    {
        base_cycle = (gimbal.base_yaw.targetAngle / 360.f) + 0.5f;
    }
    else
    {
        base_cycle = (gimbal.base_yaw.targetAngle / 360.f) - 0.5f;
	}
    float base_target_unwrap = base_cycle * 360.f + target_base;//找圈数
    gimbal.base_yaw.targetAngle = base_target_unwrap;// 大yaw直接瞄就行

    float top_target_unwrap =base_target_unwrap + (target_top - target_base);
	float delta = top_target_unwrap - gimbal.base_yaw.totalAngle;//计算相对误差
	LIMIT(delta,-TOP_YAW_LIMIT,TOP_YAW_LIMIT);//delta限幅
	gimbal.top_yaw.targetAngle = gimbal.base_yaw.totalAngle + delta;
	gimbal.top_pitch.targetAngle = vision.pitch;
    // 小pitch限位：相对于fold_pitch的相对角度限制
    float relativeAngle = gimbal.top_pitch.targetAngle - gimbal.fold_pitch.targetAngle;
    LIMIT(relativeAngle, gimbal.top_pitch.relativePitchMin, gimbal.top_pitch.relativePitchMax);
    gimbal.top_pitch.targetAngle = gimbal.fold_pitch.targetAngle + relativeAngle;
    // 大pitch限位：绝对角度限制 (目前仍存在问题 底盘上坡时如果折叠pitch处于最低,上坡时底盘会和大pitch打架)
    LIMIT(gimbal.fold_pitch.targetAngle, gimbal.fold_pitch.pitchMin, gimbal.fold_pitch.pitchMax);
}

void Gimbal_ScanCtrl()
{
    //纯累加 dt = 1ms
    float yaw_delta = gimbal.scan.yaw_speed * 0.001f;
    gimbal.top_yaw.targetAngle += yaw_delta;
    gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle;

    gimbal.scan.pitch_phase += 2.0f * PI * gimbal.scan.pitch_freq * 0.001f;
    if (gimbal.scan.pitch_phase > 2.0f * PI) {
        gimbal.scan.pitch_phase -= 2.0f * PI;
    }

    float current_ideal_pitch = gimbal.scan.pitch_amp * arm_sin_f32(gimbal.scan.pitch_phase) + gimbal.scan.pitch_offset;

    if (gimbal.scan.init_flag) {
        gimbal.scan.last_ideal_pitch = current_ideal_pitch;
        gimbal.scan.init_flag = 0;
    }
    float pitch_wave_delta = current_ideal_pitch - gimbal.scan.last_ideal_pitch;

    float error = current_ideal_pitch - gimbal.top_pitch.targetAngle;
    float convergence_step = error * 0.02f; 

    gimbal.top_pitch.targetAngle += pitch_wave_delta + convergence_step;

    gimbal.scan.last_ideal_pitch = current_ideal_pitch;
}

// void Gimbal_Follow_IMU(void)
// {

//     float relative_angle_deg = gimbal.pitch.targetAngle - gimbal.pitch.angle;    
//     p_target = gimbal.pitchMotor.para.pos + (relative_angle_deg * DEG_TO_RAD * PITCH_DIRECTION);    
// //    LIMIT(p_target,MOTOR_ZERO_POS + MOTOR_ZERO_POS - gimbal.pitch.pitchMax * DEG_TO_RAD,MOTOR_ZERO_POS + gimbal.pitch.pitchMin);
// }//MOTOR_ZERO_POS

/**************freertos任务**************/
void Task_Gimbal_Callback()
{
	Gimbal_ModeCtrl();
	Gimbal_UpdatePID();
	Gimbal_UpdateAngle();

	switch(gimbal.state)
    {
        case GimbalState_Rocker:
            Gimbal_HandleRocker();
            break;

        case GimbalState_Vision:
            Gimbal_HandleVision();
            break;

        case GimbalState_Scan:
            Gimbal_HandleScan();
            break;
		case GimbalState_Fold:
			Gimbal_HandleFold();
			break;
        default:
            break;
    }

	//计算小yaw电机输出
	DEPID_CascadeCalc(&gimbal.top_yaw.imuPID,gimbal.top_yaw.targetAngle,gimbal.top_yaw.totalAngle,gimbal.top_yaw.gyro);
	gimbal.top_yaw.imuPID.output = gimbal.top_yaw.imuPID.output;
	
	// 计算大yaw电机输出
	// DEPID_CascadeCalc(&gimbal.base_yaw.imuPID, gimbal.base_yaw.targetAngle, gimbal.base_yaw.totalAngle, gimbal.base_yaw.gyro);
	// gimbal.base_yaw.imuPID.output = -gimbal.base_yaw.imuPID.output/1000.0f - 2.0f * (gimbal.top_yaw.imuPID.output / 30000.0f) - forwardfeed(gimbal.base_yaw.imuPID.outer.output / 1000.0f);//因为电机倒置 所以输出反向 输出除一千让PID参数乘1000方便调参 再加入前馈
	PID_SingleCalc(&gimbal.base_yaw.imuPID.outer,gimbal.base_yaw.targetAngle,gimbal.base_yaw.totalAngle);  //自己写位置环 速度环用mit的
	gimbal.base_yaw.imuPID.outer.output = gimbal.base_yaw.imuPID.outer.output / 1000.0f;

	// 计算顶pitch电机输出
	// DEPID_CascadeCalc(&gimbal.pitch.imuPID, gimbal.pitch.targetAngle, gimbal.pitch.angle, gimbal.pitch.gyro);
	PID_SingleCalc(&gimbal.top_pitch.imuPID.outer,gimbal.top_pitch.targetAngle,gimbal.top_pitch.angle);
	gimbal.top_pitch.imuPID.output = - gimbal.top_pitch.imuPID.output/1000.0f  - PITCH_MASS * MASS_G * PITCH_R * arm_cos_f32(gimbal.top_pitch.angle * PI / 180.0f); ////输出除一千让PID参数乘1000方便调参  同时加入前馈
	// Gimbal_Follow_IMU();

	// 计算折叠pitch电机输出
	PID_SingleCalc(&gimbal.fold_pitch.imuPID.outer,gimbal.fold_pitch.targetAngle,gimbal.fold_pitch.angle);
}


/*************云台操控任务******************/

void OS_GimbalCallback(void const *argument)
{
	osDelay(1500);
	Gimbal_Init();
	for (;;)
	{
		Task_Gimbal_Callback();
		osDelay(1);
	}
}

