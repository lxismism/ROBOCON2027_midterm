/**
 * @file debug_task2.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "debug1_task.h"
#include "debug2_task.h"
#include "topics.hpp"

osThreadId_t Debug2_TaskHandle;


void debug2Task(void *argument){
    TickType_t currentTime = xTaskGetTickCount();

    for(;;){

        vTaskDelayUntil(&currentTime, 5);
    }

}
