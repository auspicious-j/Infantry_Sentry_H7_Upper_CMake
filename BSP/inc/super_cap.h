#ifndef __SUPER_CAP_H
#define __SUPER_CAP_H

#include "main.h"

#define CAP_CANID 0x1aa

typedef struct
{
	struct
	{
		uint16_t cap_vot;
//		int16_t total_output_power;
//		uint8_t power_ctrl_mode; //2-charge 1-output 0-turnoff
//		int16_t cap_output_power;

//	数值 需要/100；
		int16_t max_power;	//cap_voltage*9
		int16_t cap_voltage;
		int16_t bus_power;
		int16_t L_current;
//	状态变量
		uint8_t power_ctrl_mode;	//0 1 2
		uint8_t automode_stage;	//0 1 2
	}receive_data;
	uint8_t targetI;
	float target_output_power;
	float cap_vot;
	float energy;
	float target_charge_power;
	float per_energy;
	float total_output;
	float cap_output;
	uint8_t power_ctrl_mode;
	uint8_t last_power_management;
} SuperCap;

void Cap_AnalysisData(void);
void Cap_CanSendData(void);

#define CAP_SET_POWER(x) (cap.target_power = x)
#define CHARGE 2
#define OUTPUT 1
#define TURNOFF 0

extern SuperCap cap;




#endif 
