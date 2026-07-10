#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "USER_RC.h"
#include "USER_Moto.h"
#include "stdbool.h"
#include "stdint.h" 
#include "slope.h"
#include "PID.h"
#include "Filter.h"

#define TOP_YAW_OFFSET 2376   //此处校准小yaw 7869 2376 5115 3740 6444
#define INIT_YAW_ANGLE -28.0f   //此处校准大yaw
#define TOP_PITCH_OFFSET 0.151818514f //此处校准pitch
#define FOLD_PITCH_OFFSET -0.926492786f //此处校准fold_pitch 校准效果为 当fold_pitch竖直向上时 角度为90度-0.826492786 //目前无过0检测 需机械安装时避开0点
// #define FOLD_PITCH_OFFSET 1.53913379f //此处校准fold_pitch

#define MASS_G 9.81f

#define TOP_PITCH_DIRECTION 1
#define TOP_PITCH_MASS 1.237f
#define TOP_PITCH_R 0.08587f   // pitch中心及其距离

#define FOLD_PITCH_DIRECTION -1
#define FOLD_PITCH_MASS 1.491f
#define FOLD_PITCH_R 0.11104f    // fold_pitch中心及其距离


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
    // GimbalState_Fold          //折叠
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
      CascadeMPC_PID imuPID_MPC;
    } top_yaw;
    struct
    {
      float initAngle;
      float angle, lastAngle; // pitch
      float gyro;
      float targetAngle, lastTargetAngle;
      float pitchMax, pitchMin; // 限幅 (绝对角度)
      float relativePitchMax, relativePitchMin; // 相对角度限幅 (相对于fold_pitch)
      CascadePID imuPID;        // pitch陀螺仪pid
    } top_pitch;

    struct
    {
      float initAngle;
      float IMU_angle;        // pitch的陀螺仪角度 由top_pitch的编码器角度计算得出
      float angle, lastAngle; // pitch的机械角度 由编码器加校准得出
      float gyro;
      float targetAngle, lastTargetAngle;
      float pitchMax, pitchMin; // 限幅 (绝对角度)
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
      AverFilter pitchFilter, yawFilter; // 鼠标数据均值滤波器
      float pitchDPI, yawDPI;            // 调整鼠标DPI
    } Mouse;
    struct{
      AverFilter pitch,yaw,find;
    }visionFilter;//视觉数据均值滤波器

    bool visionEnable;
    bool scan_flag;
    bool fold_flag;
} Gimbal_t;

extern Gimbal_t gimbal;
extern float p_target;
extern float visionFindAver;

void Gimbal_Init(void);
void Gimbal_RockerCtrl();
void Gimbal_VisionCtrl();
void Gimbal_MouseCtrl();
void Gimbal_ScanCtrl();
void Gimbal_FoldCtrl();
void Gimbal_Fold_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event);
#endif
