/****************PID运算****************/

#include "PID.h"

//初始化pid参数
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->deadzone=0;
}

//初始化微分先行pid参数
void DEPID_Init(DEPID *pid,float p,float i,float d,float maxI,float maxOut,float gama)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->gama=gama;
}

void PD_Init(PD_Controller *pd,float kp,float kd,float maxTorque)
{
    pd->kp = kp;
    pd->kd = kd;

    pd->deadzone = 0.0f;
    pd->torque_ff = 0.0f;

    pd->maxTorque = maxTorque;

    pd->outputTorque = 0.0f;
}


//单级微分先行pid计算
void PIDRegulation(DEPID *vPID,float reference, float feedback, float differentiation)
{
	//更新数据
	vPID->lasterror=vPID->error;
	vPID->error=reference-feedback;
	//微分滤波
	differentiation = vPID->gama * differentiation + (1-vPID->gama) * vPID-> lastPv;
	//计算微分
	vPID->output = differentiation * vPID->kd;
	//计算比例
	vPID->output+=vPID->error*vPID->kp;
	//计算积分
	vPID->integral+=vPID->error*vPID->ki;
	LIMIT(vPID->integral,-vPID->maxIntegral,vPID->maxIntegral);//积分限幅
	vPID->output+=vPID->integral;
	//输出限幅
	LIMIT(vPID->output,-vPID->maxOutput,vPID->maxOutput);
	//更新微分滤波
	vPID-> lastPv = differentiation;
}

//单级pid计算
void PID_SingleCalc(PID *pid,float reference,float feedback)
{
	//更新数据
	pid->lastError=pid->error;
	if(ABS(reference-feedback) < pid->deadzone)//若误差在死区内则error直接置0
		pid->error=0;
	else
		pid->error=reference-feedback;
	//计算微分
	pid->output=(pid->error-pid->lastError)*pid->kd;
	//计算比例
	pid->output+=pid->error*pid->kp;
	//计算积分
	pid->integral+=pid->error*pid->ki;
	LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//积分限幅
	pid->output+=pid->integral;
	//输出限幅
	LIMIT(pid->output,-pid->maxOutput,pid->maxOutput);
}

//串级pid计算
void PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
	PID_SingleCalc(&(pid->outer),angleRef,angleFdb);//计算外环(角度环)
	PID_SingleCalc(&(pid->inner),pid->outer.output ,speedFdb);//计算内环(速度环)
	pid->output=pid->inner.output;
}

//串级微分先行pid计算		适用于云台 TODO拨弹等其他电机
void DEPID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
	PIDRegulation(&(pid->deOuter),angleRef,angleFdb,-speedFdb);//计算外环微分先行(角度环)
	PID_SingleCalc(&(pid->inner),pid->deOuter.output ,speedFdb);//计算内环(速度环)
	pid->output=pid->inner.output;
}


void PD_ParallelCalc(PD_Controller *pd,float p_des,float v_des,float p_meas,float v_meas)
{
    float pos_err;
    float vel_err;
    /* 位置误差 */
    pos_err = p_des - p_meas;
    if (ABS(pos_err) < pd->deadzone)
        pos_err = 0.0f;
    /* 速度误差 */
    vel_err = v_des - v_meas;
    if (ABS(vel_err) < pd->deadzone)
        vel_err = 0.0f;
    /* 并级 PD + 前馈 */
    pd->outputTorque = pd->kp * pos_err + pd->kd * vel_err + pd->torque_ff;
    /* 力矩限幅 */
    LIMIT(pd->outputTorque,-pd->maxTorque,pd->maxTorque);
}

//清空一个pid的历史数据
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}

void DEPID_Clear(DEPID *pid)
{
	pid->error=0;
	pid->lasterror=0;
	pid->integral=0;
	pid->output=0;
	pid->lastPv=0;
}



//重新设定pid输出限幅
void PID_SetMaxOutput(PID *pid,float maxOut)
{
	pid->maxOutput=maxOut;
}

//设置PID死区
void PID_SetDeadzone(PID *pid,float deadzone)
{
	pid->deadzone=deadzone;
}


void MPC_PID_Init(MPC_PID *mpc_pid,float p,float i,float d,float maxI,float maxOut,float k)
{
	mpc_pid->kp=p;
	mpc_pid->ki=i;
	mpc_pid->kd=d;
	mpc_pid->maxIntegral=maxI;
	mpc_pid->maxOutput=maxOut;
	mpc_pid->K=k;
}
void MPC_DEPID_Init(MPC_DEPID *pid,float p,float i,float d,float maxI,float maxOut,float gama,float k)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->gama=gama;
	pid->K=k;
}
void MPC_PID_SingleCalc(MPC_PID *mpc_pid,float reference,float feedback,float target)
{
	//更新数据
	mpc_pid->lastError=mpc_pid->error;
	if(ABS(reference-feedback) < mpc_pid->deadzone)//若误差在死区内则error直接置0
		mpc_pid->error=0;
	else
	mpc_pid->error=reference-feedback;
	//计算微分
	mpc_pid->output=(mpc_pid->error-mpc_pid->lastError)*mpc_pid->kd;
	//计算比例
	mpc_pid->output+=mpc_pid->error*mpc_pid->kp;
	//计算积分
	mpc_pid->integral+=mpc_pid->error*mpc_pid->ki;
	
	LIMIT(mpc_pid->integral,-mpc_pid->maxIntegral,mpc_pid->maxIntegral);//积分限幅
	
	mpc_pid->output+=mpc_pid->integral;
	//输出限幅
	mpc_pid->output+=mpc_pid->K*target;
	LIMIT(mpc_pid->output,-mpc_pid->maxOutput,mpc_pid->maxOutput);
}
void MPC_PIDRegulation(MPC_DEPID *vPID,float reference, float feedback, float differentiation,float target)
{
	//更新数据
	vPID->lastError=vPID->error;

	vPID->error=reference-feedback;
	//微分滤波
	differentiation = vPID->gama * differentiation + (1-vPID->gama) * vPID-> lastPv;
	//计算微分
	vPID->output = differentiation * vPID->kd;
	//计算比例
	vPID->output+=vPID->error*vPID->kp;
	//计算积分
	vPID->integral+=vPID->error*vPID->ki;
	
	LIMIT(vPID->integral,-vPID->maxIntegral,vPID->maxIntegral);//积分限幅
	
	vPID->output+=vPID->integral;
	vPID->output+=vPID->K*target;
	//输出限幅
	LIMIT(vPID->output,-vPID->maxOutput,vPID->maxOutput);
	//更新微分滤波
	vPID->lastPv = differentiation;
}
//串级pid计算
void MPC_PID_CascadeCalc(CascadeMPC_PID *mpc_pid,float angleRef,float angleFdb,float speedFdb,float speedTarget,float target)
{
	MPC_PID_SingleCalc(&(mpc_pid->mpcOuter),angleRef,angleFdb,speedTarget);//计算外环(角度环)
	MPC_PID_SingleCalc(&(mpc_pid->inner),mpc_pid->mpcOuter.output ,speedFdb,target);//计算内环(速度环)
	mpc_pid->output=mpc_pid->inner.output;
}
void MPC_DEPID_CascadeCalc(CascadeMPC_PID *mpc_pid,float angleRef,float angleFdb,float speedFdb,float speedTarget,float target)
{
	MPC_PIDRegulation(&(mpc_pid->mpcdeOuter),angleRef,angleFdb,-speedFdb,speedTarget);//计算外环(角度环)
	MPC_PID_SingleCalc(&(mpc_pid->inner),mpc_pid->mpcdeOuter.output ,speedFdb,target);//计算内环(速度环)
	mpc_pid->output=mpc_pid->inner.output;
}
