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

VisionTransmit vision_transmit = {0};
VisionReceive vision_receive = {0};
Vision_Type vision = {0};

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
//	Vision_RegisterEvents();
}

//// 注册事件
//void Vision_RegisterEvents()
//{
//	// R键切换视觉模式
//	RC_Register(Key_R, CombineKey_None, KeyEvent_OnDown, Vision_Change_KeyCallback);
//	// RC_Register(Key_X,CombineKey_None,KeyEvent_OnDown,Vision_RuneDir_KeyCallback);
//	// RC_Register(Key_A,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Expo_KeyCallback);
//	// RC_Register(Key_D,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Expo_KeyCallback);
//	// RC_Register(Key_W,CombineKey_Ctrl,KeyEvent_OnDown,Vision_Change_KeyCallback);
//}

//// 切换视觉模式
//void Vision_Change_KeyCallback(KeyType key, KeyCombineType combine, KeyEventType event)
//{
//	Vision_Mode = (Vision_Mode+1) % 3;
//}

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
//	vision_transmit.header = 0x5A;
//	if(gimbal.visionEnable)
//		vision_transmit.task_mode = Vision_Mode + 1;
//	else
//		vision_transmit.task_mode = 0;
//	vision_transmit.enemy_color = !USER_JudgeData.self_color; // 打红0 打蓝1
//	vision_transmit.bullet_speed = USER_JudgeData.initial_speed; //暂时随便给上 一个数，之后再加
//	vision_transmit.roll = INS.roll/180.0f*PI;
//	vision_transmit.pitch = INS.pitch/180.0f*PI;
//	vision_transmit.pitch_vel = -INS.gyro[0];
//	vision_transmit.yaw = INS.yaw/180.0f*PI;
//	vision_transmit.yaw_vel = -INS.gyro[1];
//	
//	Append_CRC16_Check_Sum((uint8_t *)&vision_transmit, sizeof(vision_transmit));
	vision_transmit.header = 0x5A;
	vision_transmit.detect_color = !USER_JudgeData.self_color; // 打红0 打蓝1
	vision_transmit.mode = 0; //0为打装甲板 1为打符
	vision_transmit.top_yaw = gimbal.top_yaw.totalAngle;
	vision_transmit.pitch = INS.pitch;
	vision_transmit.roll = INS.roll;
	vision_transmit.diff_yaw = (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0f * 360.0f;
  	vision_transmit.diff_pitch = (TOP_PITCH_OFFSET - gimbal.top_pitchMotor.para.pos) / (2 * PI) * 360.0f;
	vision_transmit.bullet_speed = USER_JudgeData.initial_speed;
	vision_transmit.robo_status = 0xFF;  /*receive_485.Judge_Data.robo_status;*/
	memcpy(&vision_transmit.AI_Judge_data, &USER_JudgeData, sizeof(Judge_Data_e));
	vision_transmit.see_enemy = vision.tracking;
	vision_transmit.end_frame = 0xA5;
}


void Vision_DataTransmit(void)
{	
	Vision_DataUpdate();
	CDC_Transmit_HS((uint8_t*)&vision_transmit, sizeof(vision_transmit));
}

void Vision_ParseData(void)
{
	//new version
//	vision.control = vision_receive.control;
//	vision.fire_thres_yaw = vision_receive.fire_thres_yaw; // 火控阈值
//	vision.fire_thres_pitch = vision_receive.fire_thres_pitch;
//	vision.target_top_yaw = vision_receive.target_yaw;
//	vision.target_top_pitch = vision_receive.target_pitch;
//	vision.top_yaw = vision_receive.yaw/PI*180.0f;
//	vision.top_yaw_vel = vision_receive.yaw_vel;
//	vision.top_yaw_acc = vision_receive.yaw_acc;
//	vision.top_pitch = vision_receive.pitch/PI*180.0f;
//	vision.top_pitch_vel = vision_receive.pitch_vel;
//	vision.top_pitch_acc = vision_receive.pitch_acc;
//	vision.bullet_id = vision_receive.bullet_id;
//	//下位机火控
//	if(ABS(gimbal.top_pitch.angle/180*PI-vision.target_top_pitch) < vision.fire_thres_pitch && ABS(gimbal.top_yaw.angle/180*PI-vision.target_top_yaw)<vision.fire_thres_yaw){
//		cmd_fire = 1;
//	}
//	else{
//		cmd_fire = 0;
//	}
// 
//	if (cmd_fire == 1 &&shooter.fricOpenFlag == 1 && shooter.workState != TRIGGER_CONTINUE && shooter.workState != TRIGGER_CLICK && gimbal.visionEnable == true)
//	{
//		//上位机火控允许发弹
//		shooter.workState = TRIGGER;
//	}
		vision.pitch = vision_receive.pitch;
		vision.base_yaw = vision_receive.base_yaw;
		vision.top_yaw = vision_receive.top_yaw;
		vision.tracking = vision_receive.tracking;
		vision.distance = vision_receive.distance;//视觉部分解包	
	
		vx=-vision_receive.linear_y*1000; //ai部分解包 同时转换坐标系 ai坐标系下向前为x 左右为y 电控坐标系下 左右为x 前后为y
		vy=-vision_receive.linear_x*1000;
		vw=vision_receive.angular_z;
		chassis.rotate.align_yaw = vision_receive.align_yaw;
		USER_SentryCmd.sentry_mode = vision_receive.sentry_mode;
		USER_SentryCmd.energy_activation = vision_receive.energy_activation;
		USER_SentryCmd.buy_projectile = vision_receive.buy_projectile;
		USER_SentryCmd.buy_life = vision_receive.buy_life;
		USER_SentryCmd.remote_buy_bullet = vision_receive.remote_buy_bullet;
		USER_SentryCmd.remote_buy_blood = vision_receive.remote_buy_blood;

		if(vision.tracking && shooter.fricOpenFlag==1 && gimbal.visionEnable)
		{		
			float a = INS.yaw / 180.0f * PI - vision.top_yaw / 180 * PI, b = vision_receive.incident_yaw ; //a是云台角度和目标角度error  b是装甲板的入射角
			float D = vision.distance, r = vision_receive.armor_radius;
			
			if(vision.tracking == 1)//此处为电控装甲板火控 判断是否瞄到装甲板
			{
				float rcosb = r * cos(b), rsinb = r * sin(b), tana = tan(a);
				
				if (tana < rcosb / (D - rsinb) && tana > -rcosb / (D + rsinb))
				{
					shooter.workState=TRIGGER_CONTINUE;
				}
				else
				{
					shooter.workState=IDLE;
				}
			}
			else //此处为电控符火控 判断是否瞄到符
			{
				static uint8_t last_rune_number = 0;
				static uint32_t last_trigger_tick = 0;
				static bool allows_trigger = false;

				allows_trigger = allows_trigger
					|| vision_receive.rune_number != last_rune_number //如果rune_number变化了 就允许开火
					|| HAL_GetTick() - last_trigger_tick > 700; //如果上次开火已经超过1秒了 就允许开火
				
				last_rune_number = vision_receive.rune_number;

				if (allows_trigger)
				{
					float R = D / cos(a - b), y = R * sin(a), x = R * cos(b);

					if (hypotf(y, x * tan(INS.pitch / 180.0f * PI) - D * tan(vision.pitch / 180.0f * PI)) <= r)
					{
						shooter.workState=TRIGGER_CLICK;
						last_trigger_tick = HAL_GetTick();
						allows_trigger = false;
					}
					else
					{
						shooter.workState=IDLE;
					}
				}
			}
		}
	
		else
		{
			shooter.workState=IDLE;
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
