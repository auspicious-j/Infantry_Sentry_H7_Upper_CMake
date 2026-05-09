#include "bsp_can.h"
#include "fdcan.h"
#include "USER_Moto.h"
#include "Shooter.h"
#include "Gimbal.h"

CanState can_state;

/**************内部工具函数声明***********************/
void CAN1_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata);
//can1接收
void CAN2_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata);
//can2接收
void CAN3_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata);
//can3接收
/******************初始化***************************/
//can过滤器初始化

void CAN_Init()
{
	FDCAN_FilterTypeDef filter;                   	//< 声明局部变量 can过滤器结构体
	filter.IdType       = FDCAN_STANDARD_ID;       	//< id设置为标准id
	filter.FilterIndex  = 0;                      	//< 设值筛选器的编号，标准id选择0-127
	filter.FilterType   = FDCAN_FILTER_MASK;       	//< 设置工作模式为掩码模式
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 	//< 将经过过滤的数据存储到 fifo0
	filter.FilterID1    = 0x00000000;                   	//< 筛选器的id
	filter.FilterID2    = 0x00000000;
	
	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);   //< 配置过滤器	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1,FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_Start(&hfdcan1);                   //< 使能can

	HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);   //< 配置过滤器	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2,FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_Start(&hfdcan2);                   //< 使能can
	
	HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);   //< 配置过滤器	
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3,FDCAN_CFG_RX_FIFO0, 1);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_Start(&hfdcan3);                   //< 使能can
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	HAL_StatusTypeDef	if_can_get_message_ok;
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	if(hfdcan == &hfdcan1)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if(if_can_get_message_ok == HAL_OK)
		{
			
			CAN1_Rx0Callback(&rx_header,rx_data);
			
		}
		else
		{
			can_state.can1_receive_error++;
		}
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	}
	else if(hfdcan == &hfdcan2)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if(HAL_OK == if_can_get_message_ok)
		{
      CAN2_Rx0Callback(&rx_header,rx_data);  
		}
		else
		{
			can_state.can2_receive_error++;
		}
		HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	}
	else if(hfdcan == &hfdcan3)
	{
		if_can_get_message_ok = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		if(HAL_OK == if_can_get_message_ok)
		{
			CAN3_Rx0Callback(&rx_header,rx_data);   
			
		}
		else
		{
			can_state.can3_receive_error++;
		}
		HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	}
        
}

//can1接收结束中断
void CAN1_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata)
{
	uint8_t whichMotor;
	switch(rx_header->Identifier)
	{		
		case 0x201:
		{
			DJIMotor_Update(&shooter.triggerMotor, (rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]), (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
			break;
		}
		//驱动电机
		case 0x202:
		case 0x203:
        {
			whichMotor = rx_header->Identifier - 0x202;
			DJIMotor_Update(&shooter.fricMotor[whichMotor], (rxdata[0]<<8 | rxdata[1]), (rxdata[2]<<8 | rxdata[3]),(rxdata[4]<<8|rxdata[5]),rxdata[6]);
			break;
		}
		//未知信息
		default:
			break;
	}
}


////can2接收结束中断
void CAN2_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata)
{
	switch(rx_header->Identifier)
	{
		case 0x11:
        {
			dm4310_fbdata(&gimbal.base_yawMotor,rxdata);
//			Detect_Update(DeviceID_Turn_Motor1+whichMotor);	
			break;
        }
		case 0x12:
        {
			dm4310_fbdata(&gimbal.top_pitchMotor,rxdata);
//			Detect_Update(DeviceID_Turn_Motor1+whichMotor);	
			break;
        }
		case 0x13:
        {
			dm4340_fbdata(&gimbal.fold_pitchMotor,rxdata);
//			Detect_Update(DeviceID_Turn_Motor1+whichMotor);	
			break;
        }
		case 0x205:
		{
			DJIMotor_Update(&gimbal.top_yawMotor,(rxdata[0] << 8 | rxdata[1]), (rxdata[2] << 8 | rxdata[3]), (rxdata[4] << 8 | rxdata[5]), rxdata[6]);
        	break;
		}
		//未知信息
		default:
			break;
	}
}
            
void CAN3_Rx0Callback(FDCAN_RxHeaderTypeDef *rx_header,uint8_t *rxdata)
{   
	switch(rx_header->Identifier)
	{
		case 0x300:
			//Supercap_Update(&cap,rxdata);
			break;
        default :
            break;
		} 
}


/********************外部调用函数*******************************/
void USER_CAN_Send(FDCAN_HandleTypeDef* hfdcan,int16_t StdId,uint8_t* tx_data)
{
	FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier = StdId;
	tx_header.IdType = FDCAN_STANDARD_ID;
 	tx_header.TxFrameType = FDCAN_DATA_FRAME;
  	tx_header.DataLength = 8; 
 	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
 	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;
		
    vTaskSuspendAll();
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,&tx_header,tx_data)!= HAL_OK)
	{
		if(hfdcan==&hfdcan1)
		{
			can_state.can1_send_error++;
		}
		else if(hfdcan==&hfdcan2)
		{
			can_state.can2_send_error++;
		}
		else if(hfdcan==&hfdcan3)
		{
			can_state.can3_send_error++;		
		}
	}
  	xTaskResumeAll();
}

static void check_can_bus(FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_ProtocolStatusTypeDef protocolStatus;

	HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
	if (protocolStatus.BusOff)
	{
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
   if (hfdcan == &hfdcan1) {
     if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {

      check_can_bus(hfdcan);
			 
    }
  }
	if (hfdcan == &hfdcan2) {
     if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {

      check_can_bus(hfdcan);
			 
    }
  }
	 if (hfdcan == &hfdcan3) {
     if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {

      check_can_bus(hfdcan);
			 
    }
  }
}
