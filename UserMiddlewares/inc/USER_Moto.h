#ifndef _USER_MOTO_H_
#define _USER_MOTO_H_

#include "main.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "PID.h"
#include "SMC.h"

//各种电机编码值与角度的换算
#define MOTOR_M3508_DGR2CODE(dgr) ((int32_t)((dgr)*436.9263f)) //3591/187*8191/360
#define MOTOR_M3508_CODE2DGR(code) ((float)((code)/436.9263f))

#define MOTOR_M2006_DGR2CODE(dgr) ((int32_t)((dgr)*819.1f)) //36*8191/360
#define MOTOR_M2006_CODE2DGR(code) ((float)((code)/819.1f))

#define MOTOR_M6020_DGR2CODE(dgr) ((int32_t)((dgr)*22.7528f)) //8191/360
#define MOTOR_M6020_CODE2DGR(code) ((float)((code)/22.7528f))

#define MIT_MODE 	0x000
#define POS_MODE	0x100
#define SPD_MODE	0x200
#define PSI_MODE    0x300

#define P_MIN -3.14159265f
#define P_MAX 3.14159265f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct DJIMOTOR
{
	int16_t angle, speed, torque;
	int8_t temp;

	int16_t lastAngle; // 记录上一次得到的角度

	int16_t targetSpeed; // 目标速度
	int32_t targetAngle; // 目标角度(编码器值)
	
	int32_t totalAngle; // 累计转过的编码器值
	
	int16_t targetCurrent;

	uint8_t ERRORFLAG;
	
	PID speedPID;		 // 速度pid(单级)
	CascadePID anglePID; // 角度pid，串级
	
	SMC FricSMC;
	
} DJI_Motor_t;

typedef struct
{
    int id;                // 电机内部设置的 can id
    int state;             // 电机状态
    int p_int;             // 整型位置信息
    int v_int;             // 整型速度信息
    int t_int;             // 整型扭矩信息
    int kp_int;            // 整型Kp信息
    int kd_int;            // 整型Kd信息
    float pos;             // 最终解析出来的位置信息
    float vel;             // 最终解析出来的速度信息
    float tor;             // 最终解析出来的扭矩信息
    float Kp;              // 最终解析出来的Kp数据
    float Kd;              // 最终解析出来的Kd数据
    float Tmos;            // 板子MOS温度
    float Tcoil;           // 电机线圈温度
} motor_fbpara_t;

typedef struct
{
		uint16_t mode;          // 电机控制模式
    	motor_fbpara_t para;  	// 电机的反馈信息结构体
	
		float totalAngle;		//电机旋转总角度 单位°
		float lastAngle;		//电机上一时刻角度 单位°
		float nowAngle;			//电机当前角度 单位°
		float targetTurnAngle;  //电机目标旋转角度  单位°
	
		PID speedPID;			  // 速度pid(单级)
		CascadePID anglePID;	  // 角度pid(串级)
	
} DM_motor_t;

typedef struct state{
    int8_t temperature;
    uint8_t error_state;
}motor_state_t;

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);

void DJIMotor_Update(DJI_Motor_t *motor, int16_t angle, int16_t speed, int16_t torque, int8_t temp);
void dm4310_fbdata(DM_motor_t *motor, uint8_t *rx_data);
void dm4340_fbdata(DM_motor_t *motor, uint8_t *rx_data);//大喵4340数据更新
void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void mit_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void USER_CAN_SetMotorCurrent(FDCAN_HandleTypeDef* hfdcan,int16_t StdId,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void Motor_StartCalcAngle(DJI_Motor_t *motor);
void Motor_CalcAngle(DJI_Motor_t *motor);

void clear_err(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id, uint16_t mode_id);

#endif
