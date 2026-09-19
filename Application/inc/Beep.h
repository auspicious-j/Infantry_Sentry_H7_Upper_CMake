#ifndef _BEEP_H_
#define _BEEP_H_

#include "main.h"

//三个八度对应的周期(us)
enum
{
	//不响
	T_None=0,
	//低八度
  T_L1=3822,
  T_L2=3405,
  T_L3=3034,
  T_L4=2863,
  T_L5=2551,
  T_L6=2272,
  T_L7=2052,
	//中八度
  T_M1=1911,
  T_M2=1703,
  T_M3=1517,
  T_M4=1432,
  T_M5=1276,
  T_M6=1136,
  T_M7=1012,
	//高八度
  T_H1=956,
  T_H2=851,
  T_H3=758,
  T_H4=716,
  T_H5=638,
  T_H6=568,
  T_H7=506
};

//各个时值对应的时长(ms) (4/4拍 速度=100)
enum
{
	D_Whole=2400,//全音符
	D_Half=1200,//二分音符
	D_Quarter=600,//四分音符
	D_Eighth=300,//八分音符
	D_Sixteenth=150//十六分音符
};

//一个音符
typedef struct
{
	uint16_t period;//发声周期
	uint16_t duration;//持续时长
}Note;

//蜂鸣器初始化
void Beep_Init(void);
//发出一个音符
void Beep_PlayOneNote(Note *note);
//发出一串音符
void Beep_PlayNotes(Note notes[],uint8_t noteNum);
//阻塞式播放所有音符
void Beep_Block_PlayAll(void);
//蜂鸣器任务回调
void Task_Beep_Callback(void);

#endif
