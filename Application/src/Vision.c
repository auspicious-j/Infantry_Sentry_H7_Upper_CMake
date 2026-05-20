#include "vision.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "chassis.h"
#include "gimbal.h"
#include "imu_temp_ctrl.h"
#include "shooter.h"
#include "chassis.h"
#include "Judge.h"
#include "USER_B2B.h"
#include "Crc.h"
#include "USER_Detcet.h"

VisionTransmit vision_transmit;
VisionReceive vision_receive;
Vision_Type vision;

uint8_t Vision_Mode = 0; // 0为空闲，1为装甲板，2为小符，3为大符
uint8_t cmd_fire = 0;

VisionSensorInfo vision_sensor_info = {
	.yaw = 0,
	.pitch = 0,
	.found = 0,
	.fire = 0,
};

VisionSensor vision_sensor = {
	.sent_info = &vision_sensor_info, // 数据结构体
	.Init = Vision_Init,			  // 传感器初始化
	.Update = Vision_DataUpdate,
	.DataReceive = Vision_DataReceive,
	.Data_Transmit = Vision_DataTransmit,
};

void Vision_Init(void)
{
	vision_transmit.header = VISION_FRAME_HEADER_TX;
	Vision_RegisterEvents();
}

// 注册事件
void Vision_RegisterEvents()
{
	// R键切换视觉模式
	RC_Register(Key_R, CombineKey_None, KeyEvent_OnDown, Vision_Change_KeyCallback);
	// RC_Register(Key_X,CombineKey_None,KeyEvent_OnDown,Vision_RuneDir_KeyCallback);
	// RC_Register(Key_A,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Expo_KeyCallback);
	// RC_Register(Key_D,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Expo_KeyCallback);
	// RC_Register(Key_W,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Change_KeyCallback);
}

// 切换视觉模式
void Vision_Change_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
{
	Vision_Mode = (Vision_Mode+1) % 4;
}

//接收来自视觉的信息
void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length)
{
	if (read_from_usart == NULL)
		return;
	// 查找帧头
	while (length) {
		if (*read_from_usart != VISION_FRAME_HEADER_RX) {
			++read_from_usart;
			--length;
		} 
		else
		{
			break;
		}
	}
	if (length == 0)
		return;
	//判断帧头数据是否正确
	if(read_from_usart[0] == VISION_FRAME_HEADER_RX)
	{
		//将数据存入接收buffer
		memcpy(&vision_receive, read_from_usart, sizeof(vision_receive));
		Vision_ParseData();
    Detect_Update(DeviceID_PC);
	}
}

//对发送的数据更新
void Vision_DataUpdate(void)
{	
	vision_transmit.header = 0x5A;
	vision_transmit.task_mode = Vision_Mode;
	vision_transmit.enemy_color = 0; // 打红0 打蓝1
	vision_transmit.bullet_speed = 22; //暂时随便给上 一个数，之后再加
	vision_transmit.roll = INS.roll/180.0f*PI;
	vision_transmit.pitch = INS.pitch/180.0f*PI;
	vision_transmit.pitch_vel = -INS.gyro[0];
	vision_transmit.yaw = INS.yaw/180.0f*PI;
	vision_transmit.yaw_vel = -INS.gyro[1];
	
	Append_CRC16_Check_Sum((uint8_t *)&vision_transmit, sizeof(vision_transmit));
}


void Vision_DataTransmit(void)
{	
	Vision_DataUpdate();
	CDC_Transmit_HS((uint8_t*)&vision_transmit, sizeof(vision_transmit));
}

void Vision_ParseData(void)
{
	
	//new version
	vision.control = vision_receive.control;
	vision.fire_thres_yaw = vision_receive.fire_thres_yaw; // 火控阈值
	vision.fire_thres_pitch = vision_receive.fire_thres_pitch;
	vision.target_top_yaw = vision_receive.target_yaw;
	vision.target_top_pitch = vision_receive.target_pitch;
	vision.top_yaw = vision_receive.yaw/PI*180.0f;
	vision.top_yaw_vel = vision_receive.yaw_vel;
	vision.top_yaw_acc = vision_receive.yaw_acc;
	vision.top_pitch = vision_receive.pitch/PI*180.0f;
	vision.top_pitch_vel = vision_receive.pitch_vel;
	vision.top_pitch_acc = vision_receive.pitch_acc;
	vision.bullet_id = vision_receive.bullet_id;
	//键鼠开自瞄
	if (rcInfo.mouse.r == 1 && visionFindAver>=0.5f)
	{
		gimbal.visionEnable = true;
	}
	//下位机火控
	if(ABS(gimbal.top_pitch.angle/180*PI-vision.target_top_pitch) < vision.fire_thres_pitch && ABS(gimbal.top_yaw.angle/180*PI-vision.target_top_yaw)<vision.fire_thres_yaw){
		cmd_fire = 1;
	}
	else{
		cmd_fire = 0;
	}
 
	if (cmd_fire == 1 &&shooter.fricOpenFlag == 1 && shooter.workState != TRIGGER_CONTINUE && shooter.workState != TRIGGER_CLICK && gimbal.visionEnable == true)
	{
		//上位机火控允许发弹
		shooter.workState = TRIGGER;
	}
}


void OS_VisionCallback(void const * argument)
{
	vision_sensor.Init();
	for(;;)
	{
		vision_sensor.Data_Transmit();
		osDelay(1);
	}
}
