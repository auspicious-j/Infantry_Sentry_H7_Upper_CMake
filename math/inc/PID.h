#ifndef _USER_PID_H_
#define _USER_PID_H_

#include "stdint.h"

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#ifndef ABS
#define ABS(x) ((x)>=0?(x):-(x))
#endif

typedef struct _PID
{
	float kp,ki,kd;
	float error,lastError;//误差、上次误差
	float integral,maxIntegral;//积分、积分限幅
	float output,maxOutput;//输出、输出限幅
	float deadzone;//死区
}PID;

/*定义结构体和公用体*/
typedef struct _DEPID
{
  float kp;     //比例系数
  float ki;      //积分系数
  float kd;    //微分系数
  float lasterror;     //前一拍偏差
	float error;				//当前error
  float output;     //输出值
  float integral;   //积分值
  float derivative;      //微分项
  float lastPv;     //前一拍的测量值
  float gama;      //微分先行滤波系数
	float maxOutput; //输出限幅
	float maxIntegral;//积分限幅
}DEPID;

typedef struct _CascadePID
{
	PID inner;//内环
	PID outer;//外环
	DEPID deOuter;//外环微分先行
	float output;//串级输出，等于inner.output
}CascadePID;

typedef struct
{
    float kp;            // 位置刚度
    float kd;            // 速度阻尼

    float deadzone;      // 死区

    float torque_ff;     // 前馈力矩 T_ff (Nm)

    float maxTorque;     // 最大力矩限制
    float outputTorque;  // 输出力矩 T_ref
} PD_Controller;

typedef struct
{
	float kp,ki,kd;
	float error,lastError;
	float integral,maxIntegral;
	float output,maxOutput;
	float deadzone;
	float K;
}MPC_PID;
typedef struct
{
	float kp,ki,kd;
	float error,lastError;
	float integral,maxIntegral;
	float output,maxOutput;
	float derivative;      //微分项
  float lastPv;     //前一拍的测量值
  float gama;      //微分先行滤波系数
	float deadzone;
	float K;
}MPC_DEPID;
typedef struct
{
	MPC_PID inner;
	MPC_PID mpcOuter;
	MPC_DEPID mpcdeOuter;
	float output;
}CascadeMPC_PID;


void PID_Init(PID *pid,float p,float i,float d,float maxSum,float maxOut);
void PID_SingleCalc(PID *pid,float reference,float feedback);
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);
void PID_Clear(PID *pid);
void DEPID_Clear(DEPID *pid);
void PID_SetMaxOutput(PID *pid,float maxOut);
void PID_SetDeadzone(PID *pid,float deadzone);
void DEPID_Init(DEPID *pid,float p,float i,float d,float maxI,float maxOut,float gama);
void PIDRegulation(DEPID *vPID,float reference, float feedback, float differentiation);
void DEPID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);
void PD_Init(PD_Controller *pd,float kp,float kd,float maxTorque);
void PD_ParallelCalc(PD_Controller *pd,float p_des,float v_des,float p_meas,float v_meas);
void MPC_PID_Init(MPC_PID *mpc_pid,float p,float i,float d,float maxI,float maxOut,float k);
void MPC_DEPID_Init(MPC_DEPID *pid,float p,float i,float d,float maxI,float maxOut,float gama,float k);
void MPC_PID_SingleCalc(MPC_PID *mpc_pid,float reference,float feedback,float target);
void MPC_PIDRegulation(MPC_DEPID *vPID,float reference, float feedback, float differentiation,float target);
void MPC_PID_CascadeCalc(CascadeMPC_PID *mpc_pid,float angleRef,float angleFdb,float speedFdb,float speedTarget,float target);
void MPC_DEPID_CascadeCalc(CascadeMPC_PID *mpc_pid,float angleRef,float angleFdb,float speedFdb,float speedTarget,float target);

#endif
