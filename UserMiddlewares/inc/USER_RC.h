#ifndef _USER_RC_H_
#define _USER_RC_H_
#include "USER_RC.h"
#include "stdint.h"
#include "usart.h"
#include "main.h"
#include "stdbool.h"
#include <stdint.h>

#define USER_RC_TYPE_DR16  1
#define USER_RC_TYPE_MC6C  2
#define USER_RC_TYPE_ET08  3

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

void RC_Init(void);

extern DMA_HandleTypeDef hdma_uart5_rx;
extern uint8_t usart5RxBuf[25]; // 串口5缓冲区
extern MC6C_RC_t rcInfo_MC6C;
extern DR16_RC_T rcInfo_DR16;
extern ET08_RC_t rcInfo_ET08; 
extern RC_TypeDef rcInfo;

#endif
