#include "Moto.h"
#include "Gimbal.h"
#include "PID.h"
#include "bsp_can.h"
#include "USER_Moto.h"
#include "fdcan.h"
#include "shooter.h"
#include "gimbal.h"
#include "judge.h"


void Motor_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t state)
{
    if(state == 0)
        enable_motor_mode(hfdcan, id, MIT_MODE);
    else if(state != 1)
        clear_err(hfdcan, id, MIT_MODE);
}

void Task_CANMotors_Callback()
{
		PID_SingleCalc(&shooter.fricMotor[0].speedPID,shooter.fricMotor[0].targetSpeed,shooter.fricMotor[0].speed);
		PID_SingleCalc(&shooter.fricMotor[1].speedPID,shooter.fricMotor[1].targetSpeed,shooter.fricMotor[1].speed);
		Motor_CalcAngle(&shooter.triggerMotor);
		PID_CascadeCalc(&shooter.triggerMotor.anglePID,shooter.triggerMotor.targetAngle,shooter.triggerMotor.totalAngle,shooter.triggerMotor.speed);

		USER_CAN_SetMotorCurrent(&hfdcan1,0x200,shooter.triggerMotor.anglePID.output,shooter.fricMotor[0].speedPID.output,shooter.fricMotor[1].speedPID.output,0);
		USER_CAN_SetMotorCurrent(&hfdcan2,0x1FF,gimbal.top_yaw.imuPID.output,0,0,0);
        // mit_ctrl(&hfdcan2,0x02,0,0,0,0,gimbal.pitch.imuPID.output);  //纯力矩控制
        mit_ctrl(&hfdcan2,0x01,0,gimbal.base_yaw.imuPID.outer.output,0,2,0);
  		mit_ctrl(&hfdcan2,0x02,0,gimbal.top_pitch.imuPID.outer.output,0,1.5,0); //mit速度环 自己写位置环 +力矩前馈
        mit_ctrl(&hfdcan2,0x03,0,gimbal.fold_pitch.imuPID.outer.output,0,1.5,0);
        // mit_ctrl(&hfdcan2,0x02,0,0,0,0,0);
        // mit_ctrl(&hfdcan2,0x03,0,0,0,0,0);

        // USER_CAN_SetMotorCurrent(&hfdcan2, 0x1FF, 0, 0, 0, 0);
}


void Task_ClearError_Callback()
{
  Motor_ClearErr(&hfdcan2,0x01,gimbal.base_yawMotor.para.state);
  Motor_ClearErr(&hfdcan2,0x02,gimbal.fold_pitchMotor.para.state);
  Motor_ClearErr(&hfdcan2,0x03,gimbal.top_pitchMotor.para.state);
}

void OS_MotorCallback(void const * argument)
{
    osDelay(1000);
    enable_motor_mode(&hfdcan2, 0x01, MIT_MODE);
    enable_motor_mode(&hfdcan2, 0x02, MIT_MODE);
    enable_motor_mode(&hfdcan2, 0x03, MIT_MODE);
    for (;;)
    {
		Task_CANMotors_Callback();
        Task_ClearError_Callback();
        osDelay(1);  // 1ms循环
    }
}

