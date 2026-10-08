/**
 * @file bsp_dwt.h
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef BSP_DWT_H
#define BSP_DWT_H

#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"
#include <stdint.h>
#include "core_cm7.h"

typedef struct{
    uint32_t s;
    uint32_t ms;
    uint32_t us;
}DWT_Typedef;

void DWT_Init(uint32_t CPU_Freq_MHz);
void DWT_Delay(float Delay);
float DWT_GetDeltaT(uint32_t *cnt_last);


#ifdef __cplusplus
}
#endif

#endif