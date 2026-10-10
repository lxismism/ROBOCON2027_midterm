/**
 * @file chassis_task.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "chassis_task.h"
#include "chassis_solution.hpp"
#include "MotorBase.hpp"
#include "com_config.h"
#include "topics.hpp"
#include "topic_pool.h"

osThreadId_t Chassis_TaskHandle;

extern std::array<MotorBase*, 4> chassis_dirmotors;
extern std::array<MotorBase*, 4> chassis_drivemotors;
SteerChassis SteerChassis_solver(chassis_dirmotors, chassis_drivemotors);

static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8U);
static pub_chassis_cmd chassis_cmd{};

bool flag = false;

void chassisTask(void *argument){
    
    TickType_t current = xTaskGetTickCount();

    for(;;)
    {
        if(chassis_cmd_sub.TryGet(&chassis_cmd)){
            SteerChassis_solver.run(chassis_cmd);
        }
        vTaskDelayUntil(&current, 1);
    }
}