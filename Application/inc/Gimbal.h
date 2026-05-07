#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "USER_Moto.h"
#include "stdbool.h"
#include "stdint.h"
#include "slope.h"
#include "PID.h"
#include "Filter.h"

#define TOP_YAW_OFFSET 7869  //此处校准小yaw
#define INIT_YAW_ANGLE -33.0f   //此处校准大yaw
#define TOP_PITCH_OFFSET -2.16f //此处校准pitch

#define PITCH_DIRECTION -1
#define MASS_G 9.81f
#define PITCH_MASS 1.3f
#define PITCH_R 0.07287f   // pitch中心及其距离




typedef enum
{
    GimbalCtrl_Control = 0,    //人控
    GimbalCtrl_AI,            //AI
} Gimbal_CtrlMode_e;
                 
typedef enum
{
    GimbalState_Rocker = 0,   //人控
    GimbalState_Vision,       //视觉
    GimbalState_Scan,         //扫描
    GimbalState_Fold          //折叠
} Gimbal_State_e;

typedef struct
{
    Gimbal_CtrlMode_e ctrl_mode;
    Gimbal_State_e state;
    struct
    {
      float initAngle;                                // yaw
      float angle, lastAngle, totalAngle, totalRound; // 用于角度统计
      float gyro;
      float targetAngle, lastTargetAngle;
      CascadePID imuPID; // yaw陀螺仪pid
    } base_yaw;
    struct
    {
      float initAngle;                                // yaw
      float angle, lastAngle, totalAngle, totalRound; // 用于角度统计
      float gyro;
      float targetAngle, lastTargetAngle;
      CascadePID imuPID; // yaw陀螺仪pid
    } top_yaw;
    struct
    {
      float initAngle;
      float angle, lastAngle; // pitch
      float gyro;
      float targetAngle, lastTargetAngle;
      float pitchMax, pitchMin; // 限幅
      CascadePID imuPID;        // pitch陀螺仪pid
    } top_pitch;

    struct
    {
      float initAngle;
      float angle, lastAngle; // pitch
      float gyro;
      float targetAngle, lastTargetAngle;
      float pitchMax, pitchMin; // 限幅
      CascadePID imuPID;        // pitch陀螺仪pid
    } fold_pitch;

    struct
    {
      float yaw_angle;        // 当前扫描角（度）
      float yaw_speed;        // 扫描速度（度/秒）
      float pitch_phase;      // 相位
      float pitch_freq;       // 频率 Hz
      float pitch_amp;        // 振幅 (5 - (-15)) / 2 = 10
      float pitch_offset;     // 中心角 (-15 + 5) / 2 = -5
      float last_ideal_pitch; // 上一次理想pitch角度，用于计算波形增量
      uint8_t init_flag;
    } scan;

    DM_motor_t base_yawMotor;
    DJI_Motor_t top_yawMotor;
    DM_motor_t top_pitchMotor;
    DM_motor_t fold_pitchMotor;

    struct{
      AverFilter pitch,yaw,find;
    }visionFilter;//视觉数据均值滤波器

    bool visionEnable;
    bool scan_flag;
} Gimbal_t;

extern Gimbal_t gimbal;
extern float p_target;

void Gimbal_Init(void);
void Gimbal_RockerCtrl();
void Gimbal_VisionCtrl();
void Gimbal_ScanCtrl();
void Gimbal_FoldCtrl();

#endif
