#include "super_cap.h"
#include "UserFreertos.h"
#include <stdio.h>

SuperCap cap;
//extern ext_game_robot_status_t			  	GameRobotStat;
void Cap_AnalysisData()
{	
//	if(GameRobotStat.power_management_chassis_output==0)
//	{
//		cap.power_ctrl_mode = TURNOFF;
//		cap.target_output_power=0;
//		cap.target_charge_power=0;
//		chassis.move.outputSlope.value=0;
//		chassis.move.chargeSlope.value=0;
//		cap.last_power_management = 0;
//	}
//	if(GameRobotStat.power_management_chassis_output==1 && cap.last_power_management == 0)
//	{
//		uint8_t lost = 0;
//		lost += Detect_IsDeviceLost(DeviceID_ChassisMotor1);
//		lost += Detect_IsDeviceLost(DeviceID_ChassisMotor2);
//		lost += Detect_IsDeviceLost(DeviceID_ChassisMotor3);
//		lost += Detect_IsDeviceLost(DeviceID_ChassisMotor4);
//		if(lost!=0)
//		{
//		cap.power_ctrl_mode = TURNOFF;
//		cap.target_output_power=0;
//		cap.target_charge_power=0;
//		chassis.move.outputSlope.value=0;
//		chassis.move.chargeSlope.value=0;
//		Cap_CanSendData();	
//		}
//		if(lost==0)
//		cap.last_power_management = 1;		
//	}
		cap.cap_vot = cap.receive_data.cap_vot / 1000.f;
		cap.energy = (cap.cap_vot*cap.cap_vot - 14.5f*14.5f) * 0.5f * 5.f;
		cap.per_energy = cap.energy/1072.5f*100.0f;  //相对7v的百分比能量值 877.5=0.5*5*（25*25-8*8）
//		cap.total_output = cap.receive_data.total_output_power/100.f;
//		cap.cap_output = cap.receive_data.cap_output_power/100.f;
}
void OS_SuperCapCallback(void const * argument)
{
  osDelay(1500);
  for(;;)
  {  
// 		Cap_AnalysisData(); 		
//    Cap_CanSendData();
    osDelay(1000);
  }

}


