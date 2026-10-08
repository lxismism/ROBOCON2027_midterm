/**
 * @file com_config.h
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */
 
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------function----------------------------------*/
uint8_t comServiceInit();
void uart3RxProcessTask(void *argument);

void can1SendTask(void *argument);
void can2SendTask(void *argument);
void can3SendTask(void *argument);


#ifdef __cplusplus
}
#endif