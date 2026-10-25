/**
 * @file pickhand_task.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "pickhand_task.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include "pickhand_solution.hpp"

#include "topic_pool.h"
#include "topics.hpp"

osThreadId_t pickHand_TaskHandle;

extern C620Motor lift_motor;
extern C620Motor tripod_motor;
extern C610Motor extend_motor;
extern DM4310Motor wrist_motor;
extern BoolDevice pump;
extern BoolDevice valve;

PickHand pickhand_solver(&lift_motor, &tripod_motor, 
    &extend_motor, &wrist_motor, &pump, &valve);

static TypedTopicSubscriber<pub_pickhand_cmd> pickhand_cmd_sub("pickhand_cmd", 8U);
static pub_pickhand_cmd pickhand_cmd{};

void pickHandTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){
            
        if(pickhand_cmd_sub.TryGet(&pickhand_cmd)){
            pickhand_solver.run(pickhand_cmd);
        }
        vTaskDelayUntil(&current, 1);
    }
}