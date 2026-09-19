#ifndef _USER_FREERTOS_H_
#define _USER_FREERTOS_H_
#include "cmsis_os.h"

extern osThreadId defaultTaskHandle;
extern osThreadId LEDTaskHandle;
extern osThreadId BeepTaskHandle;
extern osThreadId MotorTaskHandle;
extern osThreadId ErrorTaskHandle;

#endif
