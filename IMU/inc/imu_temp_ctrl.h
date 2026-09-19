#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H

typedef struct
{
    float q[4]; // 四元数估计值
    float gyro[3];  // 角速度
    float accel[3]; // 加速度
		float temp;			// 温度
    // 位姿
    float roll;
    float pitch;
    float yaw;
    float YawTotalAngle;
} INS_t;

extern INS_t INS;

void INS_Init(void);
void IMU_task(void * argument);

#endif // IMU_TEMP_CTRL_H
