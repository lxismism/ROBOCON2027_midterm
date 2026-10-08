/**
 * @file debug_task1.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief APP调试层，专门用来测试程序
 * @version 0.1
 * @date 2026-09-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "debug1_task.h"
#include "cmsis_os2.h"
#include "memory_map.h"
#include "double_buffer.hpp"
#include "lockfree_queue.hpp"
#include "UartPort.hpp"
#include "topics.hpp"
#include "Canbus.hpp"
#include "Motor.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "chassis_solution.hpp"
#include "chassis_task.h"

osThreadId_t Debug1_TaskHandle;



void debug1Task(void *argument)
{
    TickType_t currentTime = xTaskGetTickCount();
    

    for(;;)
    {


        vTaskDelayUntil(&currentTime, 1);
    }

}