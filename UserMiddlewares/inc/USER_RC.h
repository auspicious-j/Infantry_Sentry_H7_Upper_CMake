#ifndef _USER_RC_H_
#define _USER_RC_H_
#include "stdint.h"
#include "usart.h"
#include "main.h"
#include "stdbool.h"
#include <stdint.h>

#define USER_RC_TYPE_DR16  1
#define USER_RC_TYPE_MC6C  2
#define USER_RC_TYPE_ET08  3
#define KEY_NUM 18
#define MAX_KEY_CALLBACK_NUM 10

#define Sentry_Mode 1
/*
 * Change USER_RC_TYPE to switch remote controller type.
 * Available values:
 *   USER_RC_TYPE_DR16
 *   USER_RC_TYPE_MC6C
 *   USER_RC_TYPE_ET08
 */
#ifndef USER_RC_TYPE
#define USER_RC_TYPE USER_RC_TYPE_DR16
#endif

#if (USER_RC_TYPE != USER_RC_TYPE_DR16) && \
    (USER_RC_TYPE != USER_RC_TYPE_MC6C) && \
    (USER_RC_TYPE != USER_RC_TYPE_ET08)
#error "Invalid USER_RC_TYPE"
#endif

//各个键位的ID，即对应结构体在keyList中的下标
typedef enum _KeyType
{
	//可用键盘按键，顺序需与遥控器数据相同
	Key_W=1<<0,
	Key_S=1<<1,
	Key_A=1<<2,
	Key_D=1<<3,
	Key_Shift=1<<4,
	Key_Ctrl=1<<5,
	Key_Q=1<<6,
	Key_E=1<<7,
	Key_R=1<<8,
	Key_F=1<<9,
	Key_G=1<<10,
	Key_Z=1<<11,
	Key_X=1<<12,
	Key_C=1<<13,
	Key_V=1<<14,
	Key_B=1<<15,
	//鼠标左右键
	Key_Left=1<<16,
	Key_Right=1<<17,
	Key_All=0x3ffff
}KeyType;

//按键事件类型
typedef enum _KeyEventType
{
	KeyEvent_OnClick,
	KeyEvent_OnLongPress,
	KeyEvent_OnDown,
	KeyEvent_OnUp,
	KeyEvent_OnPressing//只要按下就会在每个检测周期调用一次
}KeyEventType;

//组合键类型
typedef enum _KeyCombineType
{
	CombineKey_None,
	CombineKey_Ctrl,
	CombineKey_Shift
}KeyCombineType;


//回调函数
typedef void (*KeyCallbackFunc)(KeyType,KeyCombineType,KeyEventType);

typedef enum
{
    SWITCH_UP = 2,
    SWITCH_MID = 3,
    SWITCH_DOWN = 1
} SwitchState;
typedef struct
{  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
	int16_t ch4;
  /* left and right lever information */
  uint8_t left;
  uint8_t right;
	uint8_t left_last;
	uint8_t right_last;
	
	int16_t ch[16];        // ch1 ~ ch16 左竖1 左横3  右竖2 右横0
	
	uint8_t lost;          // 失联标志
	uint8_t failsafe;      // 失控保护
} MC6C_RC_t;

typedef struct
{  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
	int16_t ch4;
  /* left and right lever information */
  uint8_t SA;
  uint8_t SB;
  uint8_t SC;
  uint8_t SD;
	
  uint8_t SA_last;
  uint8_t SB_last;
  uint8_t SC_last;
  uint8_t SD_last;

	int16_t ch[16];        // ch1 ~ ch16
	
	uint8_t lost;          // 失联标志
	uint8_t failsafe;      // 失控保护
} ET08_RC_t;


typedef struct 
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t left;
  uint8_t right;	//中间是3，上边是1，下边是2
  uint8_t left_last;
  uint8_t right_last;
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
  int16_t wheel;
}DR16_RC_T;

typedef struct //统一接口
{
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;

  uint8_t lleft; //最左边 两档开关
  uint8_t left;  //左 三挡开关
  uint8_t right; //右 三挡开关
  uint8_t rright;//最右边 两档开关

  uint8_t lleft_last; //开关旧状态
  uint8_t left_last;
  uint8_t right_last;
  uint8_t rright_last;

  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;

  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;

  int16_t wheel;
  int16_t ch[16];

  uint8_t lost;
  uint8_t failsafe;
} RC_TypeDef;

typedef struct 
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
	int16_t wheel;
  /* left and right lever information */
  uint8_t pause;
	uint8_t left;
  uint8_t right;
	uint8_t sw;
	uint8_t trigger;
	
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t left;
    uint8_t right;
		uint8_t middle;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
}Image_Trans_TypeDef;

//按键结构体，用于计算键盘/鼠标的按键事件
typedef struct _Key
{
	//需要配置的参数
	uint16_t clickDelayTime;//按下多久才算单击一次
	uint16_t longPressTime;//按下多久才算长按
	
	//用来使用的参数，仅在对应条件有效的一瞬间为1
	uint8_t isClicked;
	uint8_t isLongPressed;
	uint8_t isUp;
	uint8_t isPressing;
	
	//回调
	struct
	{
		KeyCombineType combineKey[MAX_KEY_CALLBACK_NUM];//组合键类型列表
		KeyCallbackFunc func[MAX_KEY_CALLBACK_NUM];//回调函数列表
		uint8_t number;//已注册的回调个数
	}onClickCb,onLongCb,onDownCb,onUpCb,onPressCb;//四种按键事件的回调
	
	//中间变量
	uint8_t lastState;//1/0为按下/松开
	uint32_t startPressTime;
}Key;

//RC初始化
void RC_Init(void);
//注册一个按键回调
void RC_Register(uint32_t key,KeyCombineType combine,KeyEventType event,KeyCallbackFunc func);

extern DMA_HandleTypeDef hdma_uart5_rx;
extern uint8_t usart5RxBuf[25]; // 串口5缓冲区
extern MC6C_RC_t rcInfo_MC6C;
extern DR16_RC_T rcInfo_DR16;
extern ET08_RC_t rcInfo_ET08; 
extern RC_TypeDef rcInfo;
extern bool Rocker_Ctrl;

#endif
