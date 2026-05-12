#include "USER_Moto.h"
#include "math.h"

/********************积累DJI电机累计角度************************/
void Motor_StartCalcAngle(DJI_Motor_t *motor)
{
	motor->totalAngle = 0;
	motor->lastAngle = motor->angle;
	motor->targetAngle = 0;
}

// 计算电机累计转过的圈数
void Motor_CalcAngle(DJI_Motor_t *motor)
{
	int32_t dAngle = 0;
	if (motor->angle - motor->lastAngle < -4000)
		dAngle = motor->angle + (8191 - motor->lastAngle);
	else if (motor->angle - motor->lastAngle > 4000)
		dAngle = -motor->lastAngle - (8191 - motor->angle);
	else
		dAngle = motor->angle - motor->lastAngle;
	// 将角度增量加入计数器
	motor->totalAngle += dAngle;
	// 记录角度
	motor->lastAngle = motor->angle;
}


int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void DJIMotor_Update(DJI_Motor_t *motor, int16_t angle, int16_t speed, int16_t torque, int8_t temp) //大疆电机数据更新
{
	motor->angle = angle;
	motor->speed = speed;
	motor->torque = torque;
	motor->temp = temp;
}

void dm4310_fbdata(DM_motor_t *motor, uint8_t *rx_data)//大喵4310数据更新
{ 
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
      motor->nowAngle = (float)motor->para.pos / M_PI * 180.0f;
}

void dm4340_fbdata(DM_motor_t *motor, uint8_t *rx_data)//大喵4340数据更新
{ 
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
      motor->nowAngle = (float)motor->para.pos / M_PI * 180.0f;
}

void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id) //大喵电机使能
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;
    
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;
    
    USER_CAN_Send(hcan, id, data);
}

void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)//大喵电机失能
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;
    
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;
    
    USER_CAN_Send(hcan, id, data);
}

void mit_ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)//mit模式 控制大喵电机
{
    uint8_t data[8];
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
    kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    data[7] = tor_tmp;
    
    USER_CAN_Send(hcan, id, data);
}

void clear_err(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id, uint16_t mode_id)
{
		uint8_t data[8];
		uint16_t id = motor_id + mode_id;
		
		data[0] = 0xFF;
		data[1] = 0xFF;
		data[2] = 0xFF;
		data[3] = 0xFF;
		data[4] = 0xFF;
		data[5] = 0xFF;
		data[6] = 0xFF;
		data[7] = 0xFB;
		
		USER_CAN_Send(hfdcan, id, data);
}


//发送电机电流信息 控制DJI电机
void USER_CAN_SetMotorCurrent(FDCAN_HandleTypeDef* hfdcan,int16_t StdId,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
		uint8_t tx_data[8]={0};
		tx_data[0] = (iq1 >> 8) & 0xff;
		tx_data[1] = (iq1) & 0xff;
		tx_data[2] = (iq2 >> 8) & 0xff;
		tx_data[3] = (iq2) & 0xff;
		tx_data[4] = (iq3 >> 8) & 0xff;
		tx_data[5] = (iq3) & 0xff;
		tx_data[6] = (iq4 >> 8) & 0xff;
		tx_data[7] = (iq4) & 0xff;	
		
		USER_CAN_Send(hfdcan,StdId, tx_data);
}





