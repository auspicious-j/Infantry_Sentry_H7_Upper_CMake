	#include "Gimbal.h"
	#include "PID.h"
	#include "Shooter.h"
	#include "USER_Detcet.h"
	#include "USER_RC.h"
	#include "gimbal.h"
	#include "imu_temp_ctrl.h"
	#include "vision.h"
	#include "chassis.h"
	#include "shooter.h"
	#include "arm_math.h"
	#include <math.h>
	#include <stdatomic.h>
	#include <stdbool.h>

	#define DEG_TO_RAD 0.01745329f  // PI / 180
	#define RAD_TO_DEG 57.2957795f  // 180 / PI
	#define TOP_YAW_LIMIT 45.0f
	#define BASE_YAW_J 0.04575f

	Gimbal_t gimbal;
	float visionFindAver;

	void Gimbal_InitPID(void);

	/********************初始化************************/
	// 初始化云台
	void Gimbal_Init()
	{
		gimbal.top_pitch.pitchMax = 60; // 设定pitch角度限幅 (绝对角度) (目前没有用到 小pitch限位由大小pitch的相对角度来决定)
		gimbal.top_pitch.pitchMin = -13;

		gimbal.top_pitch.initAngle = 0; // 陀螺仪pitch开机角度 *0表示pitch水平
		gimbal.top_pitch.targetAngle = gimbal.top_pitch.initAngle;

		gimbal.fold_pitch.pitchMax = 95;
		gimbal.fold_pitch.pitchMin = 5;

		gimbal.fold_pitch.initAngle = 95;
		gimbal.fold_pitch.targetAngle = gimbal.fold_pitch.initAngle;

		gimbal.base_yaw.initAngle = 0; // 陀螺仪yaw开机角度   *0表示yaw不动
		gimbal.base_yaw.targetAngle = gimbal.base_yaw.initAngle;

		gimbal.top_yaw.initAngle = 0;
		gimbal.top_yaw.targetAngle = gimbal.top_yaw.initAngle;


		Filter_InitAverFilter(&gimbal.visionFilter.find,50);
		gimbal.visionEnable = false;
		gimbal.state = GimbalState_Rocker;

		gimbal.scan.yaw_speed = 60.0f;
		gimbal.scan.pitch_phase = 0.0f;
		gimbal.scan.pitch_freq = 1.0f;
		gimbal.scan.pitch_amp = 10.0f;
		gimbal.scan.pitch_offset = -5.0f;
		gimbal.scan.last_ideal_pitch = 0.0f;
		gimbal.scan.init_flag = 1;
		Gimbal_InitPID();         // 初始化PID参数
		Gimbal_RegisterEvents();
	}

	void Gimbal_InitPID()
	{
		/*pitch由陀螺仪控制*/
		PID_Init(&gimbal.top_pitch.imuPID.inner, 6, 0, 0.4, 500, 7000);
		DEPID_Init(&gimbal.top_pitch.imuPID.deOuter, 75, 0.7, 120, 200, 7000, 0.6);//45, 0.7, 50串级pid
		PID_Init(&gimbal.top_pitch.imuPID.outer,-1,-0.001,-8,1,10); //自己写位置环 mit速度环

		PID_Init(&gimbal.fold_pitch.imuPID.outer, 1.8, 0.001, 10, 1, 15);

		PID_Init(&gimbal.base_yaw.imuPID.inner,5.8,0.02,3.5,1000,7000);
		DEPID_Init(&gimbal.base_yaw.imuPID.deOuter,37,0.03,173,700,2000,0.45);	//串级pid
		PID_Init(&gimbal.base_yaw.imuPID.outer,330,0.1,11000,1000,30000);//自己写位置环 mit速度环

		PID_Init(&gimbal.top_yaw.imuPID.inner, 50, 0.1, 10, 7000, 30000);//270 0.3  300
		DEPID_Init(&gimbal.top_yaw.imuPID.deOuter, 280, 0.5, 480, 100, 30000, 0.7);
//		DEPID_Init(&gimbal.top_yaw.imuPID.deOuter, 30, 0.5, 70, 100, 30000, 0.7);

	}

	/*云台角度更新*/
	void Gimbal_UpdateAngle()
	{
		// static int16_t last_angle = 0;
		// static int32_t total_angle = 0;
		// static uint8_t init = 0;

		gimbal.top_yaw.gyro = -INS.gyro[1];
		gimbal.top_yaw.angle = INS.yaw;

		// if(!init && detectList[DeviceID_TopYawMotor].isLost == 0)//当yaw不掉线且第一次进入
		// {
		// 	last_angle = gimbal.top_yawMotor.angle;
		// 	total_angle = gimbal.top_yawMotor.angle;
		// 	init = 1;
		// }
		// int16_t delta = gimbal.top_yawMotor.angle - last_angle; //加入过0检测
		// if(delta > 4096)      delta -= 8192;
		// else if(delta < -4096) delta += 8192;
		// total_angle += delta;
		// last_angle = gimbal.top_yawMotor.angle;
		gimbal.base_yaw.angle = gimbal.top_yaw.angle - (/*total_angle*/ gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0f * 360.0f; //得出大yaw的imu角度

		gimbal.base_yaw.gyro = gimbal.base_yawMotor.para.vel;
		gimbal.top_pitch.gyro = -INS.gyro[0];
		gimbal.top_pitch.angle = INS.pitch;
		gimbal.fold_pitch.angle = gimbal.fold_pitchMotor.nowAngle - FOLD_PITCH_OFFSET * RAD_TO_DEG;//根据折叠pitch的电机编码器值控制
		gimbal.fold_pitch.IMU_angle = gimbal.top_pitch.angle + (gimbal.top_pitchMotor.para.pos - TOP_PITCH_OFFSET) * RAD_TO_DEG + 90.0f;//根据顶上pitch的imu角度和电机编码器值得出折叠pitch的imu角度
		gimbal.fold_pitch.gyro = gimbal.fold_pitchMotor.para.vel;
		chassis.angle.pitchTiltAngle = gimbal.top_pitch.angle + (180.0f - (-gimbal.top_pitchMotor.nowAngle + TOP_PITCH_OFFSET * RAD_TO_DEG + 90.0f)) - gimbal.fold_pitch.angle;//根据顶上pitch的imu角度和电机编码器值得出底盘的pitch倾斜角度

		//pitch角度归一化到-180~180度
		if (gimbal.fold_pitch.angle > 180.0f)
			gimbal.fold_pitch.angle -= 360.0f;
		else if (gimbal.fold_pitch.angle < -180.0f)
			gimbal.fold_pitch.angle += 360.0f;

		//更新top_pitch限幅 根据fold_pitch角度动态限幅
		if (gimbal.fold_pitch.angle >= 60.0f && gimbal.fold_pitch.angle <= 100.0f)
		{
			// fold_pitch角度在95-60度之间时，相对角度限幅为 60到-13度
			gimbal.top_pitch.relativePitchMax = 60.0f;
			gimbal.top_pitch.relativePitchMin = -13.0f;
		}
		else if (gimbal.fold_pitch.angle >= 10.0f && gimbal.fold_pitch.angle < 60.0f)
		{
			// fold_pitch角度在70-10度之间时，映射相对角度限幅
			gimbal.top_pitch.relativePitchMax = 93.0f;
			gimbal.top_pitch.relativePitchMin = 93.0f - 18.0f * gimbal.fold_pitch.angle / 10.0f;
		}
		else if (gimbal.fold_pitch.angle < 15.0f && gimbal.fold_pitch.angle >= -5.0f)
		{
			// fold_pitch角度在0到-15度之间时，锁死角度为3度
			gimbal.top_pitch.relativePitchMax = 93.0f;
			gimbal.top_pitch.relativePitchMin = 90.0f;
		}
		else
		{
			// 其他角度范围，使用默认值
			gimbal.top_pitch.relativePitchMax = 93.0f;
			gimbal.top_pitch.relativePitchMin = -10.0f;
		}
		gimbal.top_pitch.pitchMax = gimbal.top_pitch.relativePitchMax + chassis.angle.pitchTiltAngle - (90.0f - gimbal.fold_pitch.angle);
		gimbal.top_pitch.pitchMin = gimbal.top_pitch.relativePitchMin + chassis.angle.pitchTiltAngle - (90.0f - gimbal.fold_pitch.angle);
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
		gimbal.base_yaw.totalAngle = gimbal.top_yaw.totalAngle - (/*total_angle*/gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0f * 360.0f;
		visionFindAver = Filter_AverCalc(&gimbal.visionFilter.find, vision.control);
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
				gimbal.base_yaw.imuPID.outer.maxIntegral = 12000;
				gimbal.base_yaw.imuPID.outer.ki = 0.5f;
				break;

			default:
				gimbal.base_yaw.imuPID.outer.maxIntegral = 1000;
				gimbal.base_yaw.imuPID.outer.ki = 0.1f;
				break;
		}
	}

	/****模式更改*****/
	void Gimbal_ModeCtrl(void)
	{
		gimbal.ctrl_mode = GimbalCtrl_Control;
		switch(gimbal.ctrl_mode)
		{
			case GimbalCtrl_Control: //人控模式
				if(rcInfo.wheel > 400 || rcInfo.mouse.r == 1)
				{
					gimbal.state = GimbalState_Vision;
					gimbal.visionEnable = true;
				}
				else
				{
					gimbal.state = GimbalState_Rocker;
					gimbal.visionEnable = false;
				}
				break;
			default:
				break;
		}
	}

	static void Gimbal_HandleRocker(void)
	{	
		if(Rocker_Ctrl)
			Gimbal_RockerCtrl();
		else
			Gimbal_MouseCtrl();
	}

	// static void Gimbal_HandleFold(void)
	// {
	// 	Gimbal_FoldCtrl();
	// }

	static void Gimbal_HandleVision(void)
	{
		gimbal.visionEnable = true;
		// shooter.fricOpenFlag = 0;
		if(vision.control)
			Gimbal_VisionCtrl();
		else
			Gimbal_RockerCtrl();
	}

	static void Gimbal_HandleScan(void)
	{
		gimbal.visionEnable = true;
		if(vision.control)
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

	float a;
	/*******五种控制函数*********/
	void Gimbal_RockerCtrl()
	{
		gimbal.top_yaw.targetAngle -= rcInfo.ch1 * 0.3f / 660.0f;	// yaw
		gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle;
		//小pitch
		gimbal.top_pitch.targetAngle += rcInfo.ch2 * 0.35 / 660.0f; // 旋转云台pitch
		LIMIT(gimbal.top_pitch.targetAngle, gimbal.top_pitch.pitchMin, gimbal.top_pitch.pitchMax);
		//折叠pitch
		if (rcInfo.left == 1 && rcInfo.left_last == 3 && shooter.fricOpenFlag == 0) //按键按下且之前未按下且摩擦轮未开
		{
			if (gimbal.fold_pitch.targetAngle < 50.0f){
				gimbal.fold_pitch.targetAngle = 95.0f;
				gimbal.fold_flag  = false;
			}
			else {
				gimbal.fold_pitch.targetAngle = 0.0f;
				gimbal.fold_flag = true;
			}
		}
		rcInfo.left_last = rcInfo.left;//叠史
		// 小pitch限位：相对于fold_pitch的相对角度限制
		LIMIT(gimbal.fold_pitch.targetAngle, gimbal.fold_pitch.pitchMin, gimbal.fold_pitch.pitchMax);//折叠pitch限位
	}

	// void Gimbal_FoldCtrl()
	// {
	// 	gimbal.top_yaw.targetAngle -= rcInfo.ch1 * 0.3 / 660.0f;	// yaw
	// 	gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle;
	//  gimbal.fold_pitch.targetAngle += rcInfo.ch2 * 0.35 / 660.0f; // 旋转云台pitch
	// 	LIMIT(gimbal.fold_pitch.targetAngle, gimbal.fold_pitch.pitchMin, gimbal.fold_pitch.pitchMax);//折叠pitch限位
	// 	// 小pitch限位：根据当前fold_pitch角度动态调整top_pitch的相对角度限幅
	// 	float relativeAngle = gimbal.top_pitch.targetAngle - gimbal.fold_pitch.IMU_angle;
	// 	LIMIT(relativeAngle, gimbal.top_pitch.relativePitchMin, gimbal.top_pitch.relativePitchMax);
	// 	gimbal.top_pitch.targetAngle = gimbal.fold_pitch.IMU_angle + relativeAngle;  //翻译一下就是限幅过的relative angle
	// }

	void Gimbal_MouseCtrl()
	{	
		if(gimbal.visionEnable == true){
			gimbal.Mouse.yawDPI = 0.005;
			gimbal.Mouse.pitchDPI = 0.005;
		}
		else{
			gimbal.Mouse.yawDPI = 0.0025;
			gimbal.Mouse.pitchDPI = 0.0033;
		}
		gimbal.top_yaw.targetAngle -= Filter_AverCalc(&gimbal.Mouse.yawFilter, rcInfo.mouse.x * gimbal.Mouse.yawDPI);		// yaw锟睫革拷
		gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle;
		gimbal.top_pitch.targetAngle += Filter_AverCalc(&gimbal.Mouse.pitchFilter, rcInfo.mouse.y * gimbal.Mouse.pitchDPI); // pitch锟睫革拷
		LIMIT(gimbal.fold_pitch.targetAngle, gimbal.fold_pitch.pitchMin, gimbal.fold_pitch.pitchMax);//折叠pitch限位
		LIMIT(gimbal.top_pitch.targetAngle, gimbal.top_pitch.pitchMin, gimbal.top_pitch.pitchMax);
	}

	void Gimbal_VisionCtrl()
	{
		if (isnan(vision.top_yaw) || isnan(vision.top_pitch))
			return;
		int yaw_cycle;
		if ((gimbal.top_yaw.targetAngle / 360.f) > 0)
			yaw_cycle = (gimbal.top_yaw.targetAngle / 360.f) + 0.5f;
		else
			yaw_cycle = (gimbal.top_yaw.targetAngle / 360.f) - 0.5f;

		float target_yaw;	
		if ((yaw_cycle * 360.f + vision.top_yaw) - gimbal.top_yaw.targetAngle > 180.f)
			target_yaw = yaw_cycle * 360.f + vision.top_yaw - 360.f;

		else if ((yaw_cycle * 360.f + vision.top_yaw) - gimbal.top_yaw.targetAngle < -180.f)
			target_yaw = yaw_cycle * 360.f + vision.top_yaw + 360.f;
		else
			target_yaw = yaw_cycle * 360.f + vision.top_yaw; //以上为多圈检测
	
		//以下为跟随部分
		gimbal.top_yaw.targetAngle = target_yaw;	// top_yaw 永远跟随视觉
		float yaw_diff = gimbal.top_yaw.totalAngle - gimbal.base_yaw.totalAngle;	// 计算 base_yaw 和 top_yaw 当前角度差
		while (yaw_diff > 180.f)	// 角度归一化
		yaw_diff -= 360.f;
		while (yaw_diff < -180.f)
		yaw_diff += 360.f;
		if (yaw_diff < 0)//去绝对值
			yaw_diff = -yaw_diff;
		// 超过30度 base_yaw 才开始跟随
		if (yaw_diff > 18.0f)
		{
			gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle;
		}
		gimbal.top_pitch.targetAngle = vision.top_pitch;
		LIMIT(gimbal.top_pitch.targetAngle, gimbal.top_pitch.pitchMin, gimbal.top_pitch.pitchMax);
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

	/********依旧键鼠***********/
	void Gimbal_RegisterEvents()
	{
		RC_Register(Key_C,CombineKey_None,KeyEvent_OnDown,Gimbal_Fold_KeyCallback);//一键折叠
	}

	void Gimbal_Fold_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
	{
		if(gimbal.fold_pitch.targetAngle > 75.0f)
		{
			gimbal.fold_pitch.targetAngle = 5.0f;
			gimbal.top_pitch.targetAngle = 0.0f;
      gimbal.fold_flag = true;
		}
		else if(gimbal.fold_pitch.targetAngle < 15.0f)
		{
			gimbal.fold_pitch.targetAngle = 95.0f;
      gimbal.fold_flag = false;
		}
	}
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
			// case GimbalState_Fold:
			// 	Gimbal_HandleFold();
			// 	break;
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
		gimbal.top_pitch.imuPID.output = - gimbal.top_pitch.imuPID.output/1000.0f - TOP_PITCH_DIRECTION * TOP_PITCH_MASS * MASS_G * TOP_PITCH_R * arm_cos_f32(gimbal.top_pitch.angle * PI / 180.0f); ////输出除一千让PID参数乘1000方便调参  同时加入前馈
		//此处不是outer.output 方便从纯力矩切换到mit速度模式 同时加入前馈
		// Gimbal_Follow_IMU();

		// 计算折叠pitch电机输出
		PID_SingleCalc(&gimbal.fold_pitch.imuPID.outer,gimbal.fold_pitch.targetAngle,gimbal.fold_pitch.angle);
		if(gimbal.fold_pitch.IMU_angle <= 60.0f)
		gimbal.fold_pitch.imuPID.output = - gimbal.fold_pitch.imuPID.outer.output / 1000.0f - FOLD_PITCH_DIRECTION * FOLD_PITCH_MASS * MASS_G * FOLD_PITCH_R * arm_cos_f32(gimbal.fold_pitch.IMU_angle * PI / 180.0f);
		if(gimbal.fold_pitch.IMU_angle > 60.0f && gimbal.fold_pitch.IMU_angle < 150.0f)
		gimbal.fold_pitch.imuPID.output = - gimbal.fold_pitch.imuPID.outer.output / 1000.0f - FOLD_PITCH_DIRECTION * FOLD_PITCH_MASS * MASS_G * FOLD_PITCH_R * arm_cos_f32(60.0f * PI / 180.0f);//当折叠pitch角度较大时，由于小pitch改变(懒得算重力补偿)，所以保持在60度时的重力补偿
	}


	/*************云台操控任务******************/

	void OS_GimbalCallback(void const *argument)
	{
		osDelay(1200);
		Gimbal_Init();
		for (;;)
		{
			Task_Gimbal_Callback();
			osDelay(1);
		}
	}

