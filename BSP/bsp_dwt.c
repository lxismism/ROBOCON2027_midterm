/**
 * @file bsp_dwt.c
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_dwt.h"

static uint32_t CPU_FREQ_Hz;

void DWT_Init(uint32_t CPU_Freq_MHz)
{
    if((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->LAR = 0xC5ACCE55;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    CPU_FREQ_Hz = CPU_Freq_MHz * 1000000;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;

}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    float dt = (float)(cnt_now - *cnt_last) / CPU_FREQ_Hz;
    *cnt_last = cnt_now;
    return dt;
}