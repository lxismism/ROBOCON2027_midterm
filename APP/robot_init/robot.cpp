/**
 * @file robot.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief APP调试层，专门用来测试程序
 * @version 0.1
 * @date 2026-09-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "main.h"
#include "robot.h"
#include "robot_task.h"
#include "com_config.h"
#include "topics.hpp"
#include "bsp_dwt.h"

void Robot_Init()
{
    __disable_irq();

    DWT_Init(SystemCoreClock / 1000000U);

    /*通信外设初始化*/
    if(comServiceInit() != 0){
        Error_Handler();
    }
    subsQueueInit();

    osTaskInit();

    __enable_irq();
}
