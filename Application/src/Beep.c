#include "Beep.h"
#include "myQueue.h"
#include "UserFreertos.h"
#include "tim.h"

#define SOFTWARE_DELAY(t) do{uint32_t i=(t)*12000; while(i--);}while(0)//软件延时

#define MAX_NOTE_QUEUE_SIZE 50

Queue noteQueue;//音符队列，存放将要播放的音符
Note noteQueueBuf[MAX_NOTE_QUEUE_SIZE];//队列数据

//蜂鸣器初始化
void Beep_Init()
{
   HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
	//队列初始化
	Queue_Init(&noteQueue,MAX_NOTE_QUEUE_SIZE);
	Queue_AttachBuffer(&noteQueue,noteQueueBuf,sizeof(Note));
}

//向蜂鸣器输出指定周期(us)占空比50%的方波
void Beep_Play(uint16_t period)
{
  HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_2);
	if(period!=0)
	{
    __HAL_TIM_SetAutoreload(&htim12,period);
    __HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_2,period/2);
 		HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
	}
}

//发出一个音符
void Beep_PlayOneNote(Note *note)
{
	//入队
	Queue_Enqueue(&noteQueue,note);
  xTaskNotifyGive(BeepTaskHandle);  //通知beep任务开始
}

//发出一串音符
void Beep_PlayNotes(Note notes[],uint8_t noteNum)
{
	//入队
	for(int i=0;i<noteNum;i++)
		Queue_Enqueue(&noteQueue,&notes[i]);
  xTaskNotifyGive(BeepTaskHandle);
}

//阻塞式播放所有音符(不应在任务调度正常运行时调用)
void Beep_Block_PlayAll()
{
	while(!Queue_IsEmpty(&noteQueue))//播放队列中所有音符
	{
		Note *note=Queue_Dequeue(&noteQueue);
		Beep_Play(note->period);
		SOFTWARE_DELAY(note->duration);//阻塞式延时，使用软件延时防止在中断中调用
	}
	Beep_Play(T_None);
}

//蜂鸣器任务回调
void Task_Beep_Callback()
{
	if(Queue_IsEmpty(&noteQueue))//没有音符要播放则停止任务
	{
 		Beep_Play(T_None);
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //等待任务通知
	}
	else
	{
		//播放队头音符并等待指定时值
		Note *note=Queue_Dequeue(&noteQueue);
		Beep_Play(note->period);
    osDelay(note->duration);
	}
}

/************************freertos任务***************************/
void OS_BeepCallback(void const * argument)
{
  //蜂鸣器播放启动声 [do re me so la(升)]
//	Beep_PlayNotes((Note[]){{T_M1,D_Sixteenth},{T_M2,D_Sixteenth},{T_M3,D_Sixteenth},{T_M5,D_Sixteenth},{T_M6,D_Sixteenth},{T_None,D_Quarter}},6);
	Beep_PlayNotes((Note[]){{T_M1,D_Sixteenth},{T_H1,D_Sixteenth},{T_None,D_Quarter}},3);
  for(;;)
  {
    Task_Beep_Callback();
    osDelay(20);
  }
}

