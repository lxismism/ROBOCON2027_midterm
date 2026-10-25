/**
 * @file weaponhand_task.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-23
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "weaponhand_task.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include "topics.hpp"
#include "topic_pool.h"

#include "djiMotor.hpp"
#include "weaponhand_solution.hpp"

static TypedTopicSubscriber<pub_weaponhand_cmd> weaponhand_cmd_sub("weaponhand_cmd", 8U);
static pub_weaponhand_cmd weaponhand_cmd{};

osThreadId_t weaponhand_TaskHandle;

extern C620Motor weapon_lift_motor;
extern C610Motor weapon_pick_motor;

static Weaponhand weaponhand_solution(&weapon_lift_motor, &weapon_pick_motor);


void weaponhandTask(void *argument){

    TickType_t current = xTaskGetTickCount();

    for(;;){

        if(weaponhand_cmd_sub.TryGet(&weaponhand_cmd)){
            weaponhand_solution.run(weaponhand_cmd);
        }

        vTaskDelayUntil(&current, 5);
    }
}