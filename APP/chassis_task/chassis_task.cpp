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

extern std::array<C620Motor*, kWheelNum> chassis_dirmotors;
extern std::array<VESCMotor*, kWheelNum> chassis_drivemotors;
SteerChassis SteerChassis_solver(chassis_dirmotors, chassis_drivemotors);

static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8U);
static pub_chassis_cmd chassis_cmd{};

static TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data", 8U);
static pub_imu_data imu_data{};

void chassisTask(void *argument){
    
    TickType_t current = xTaskGetTickCount();
    vTaskDelayUntil(&current, 20);

    for(;;)
    {
        if(!SteerChassis_solver.HasHoming()){
            SteerChassis_solver.homing();
        } else{
            if(chassis_cmd_sub.TryGet(&chassis_cmd)){
                imu_data_sub.TryGet(&imu_data);
                SteerChassis_solver.run(chassis_cmd, imu_data);
            }
        }    
        vTaskDelayUntil(&current, 1);
    }
}