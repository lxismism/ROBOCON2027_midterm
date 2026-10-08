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
#include "Motor.hpp"
#include "com_config.h"

osThreadId_t Chassis_TaskHandle;

extern std::array<MotorBase*, 4> chassis_dirmotors;
extern std::array<MotorBase*, 4> chassis_drivemotors;
SteerChassis SteerChassis_solver(chassis_dirmotors, chassis_drivemotors);
static pub_chassis_cmd chassis_cmd_sub{};

void chassisTask(void *argument){
    
    TickType_t current = xTaskGetTickCount();

    for(;;)
    {
        SteerChassis_solver.run(chassis_cmd_sub);
        vTaskDelayUntil(&current, 1);
    }
}