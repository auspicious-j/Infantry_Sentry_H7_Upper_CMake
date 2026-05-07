#include "bsp_ws2812.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>

/*
 * DMA1 on STM32H7 cannot access DTCM. The default linker script puts .bss into
 * DTCMRAM, so keep the WS2812 PWM DMA buffer in AXI SRAM (RAM_D1).
 */
 
__attribute__((section(".RAM_D1"), aligned(32)))
uint16_t WS2812_RGB_Buff[LED_1_NUM * DATA_LEN + WS2812_RST_NUM];

static volatile uint8_t ws2812_dma_busy = 0;

static void WS2812_WaitReady(void)
{
    uint32_t tickstart = HAL_GetTick();

    while (ws2812_dma_busy != 0U)
    {
        if ((HAL_GetTick() - tickstart) > 2U)
        {
            HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
            ws2812_dma_busy = 0U;
            break;
        }
    }
}

static void WS2812_Transmit(void)
{
    WS2812_WaitReady();

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    ws2812_dma_busy = 1U;
    if (HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)WS2812_RGB_Buff,
                              sizeof(WS2812_RGB_Buff) / sizeof(WS2812_RGB_Buff[0])) != HAL_OK)
    {
        ws2812_dma_busy = 0U;
    }
} 


void WS2812_Set(uint16_t num, uint8_t R, uint8_t G, uint8_t B)
{
    if (num >= LED_1_NUM)
    {
        return;
    }

    WS2812_WaitReady();

    uint32_t indexx = num * DATA_LEN;
    for (uint8_t i = 0; i < 8; i++)
    {
        WS2812_RGB_Buff[indexx + i]      = ((G << i) & 0x80U) ? WS_H : WS_L;
        WS2812_RGB_Buff[indexx + i + 8]  = ((R << i) & 0x80U) ? WS_H : WS_L;
        WS2812_RGB_Buff[indexx + i + 16] = ((B << i) & 0x80U) ? WS_H : WS_L;
    }

    WS2812_Transmit();
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0U);
        ws2812_dma_busy = 0U;
    }
}

void WS2812_Init(void)
{
    memset(WS2812_RGB_Buff, 0, sizeof(WS2812_RGB_Buff));
    WS2812_Set(0, 0, 0, 0);
}

void OS_LedCallback(void const * argument)
{
    WS2812_Init();
    for (;;)
    {
        WS2812_Set(0, 0, 50, 0);
        osDelay(500);
        WS2812_Set(0, 0, 0, 0);
        osDelay(500);
    }
}
