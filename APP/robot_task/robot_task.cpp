/**
 * @file robot_task.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 任务管理，创建所有任务
 * @version 0.1
 * @date 2026-09-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "robot_task.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "debug1_task.h"
#include "debug2_task.h"
#include "com_config.h"
#include "chassis_task.h"




extern osThreadId_t Debug1_TaskHandle;
extern osThreadId_t Debug2_TaskHandle;
extern osThreadId_t uart3Process_TaskHandle;
extern osThreadId_t can1Send_TaskHandle;
extern osThreadId_t can2Send_TaskHandle;
extern osThreadId_t can3Send_TaskHandle;
extern osThreadId_t Chassis_TaskHandle;


void osTaskInit(void)
{
    const osThreadAttr_t Debug1TaskHandle_attributes = {
        .name = "Debug1_TaskHandle",
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    Debug1_TaskHandle = osThreadNew(debug1Task, NULL, &Debug1TaskHandle_attributes);

    const osThreadAttr_t uart3ProcessTaskHandle_attributes = {
        .name = "uart3Process_TaskHandle",
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    uart3Process_TaskHandle = osThreadNew(uart3RxProcessTask, NULL, &uart3ProcessTaskHandle_attributes);

    const osThreadAttr_t Debug2TaskHandle_attributes = {
        .name = "Debug2_TaskHandle",
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
    };
    Debug2_TaskHandle = osThreadNew(debug2Task, NULL, &Debug2TaskHandle_attributes);

    const osThreadAttr_t can1SendTaskHandle_attributes = {
        .name = "can1Send_TaskHandle",
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityRealtime,
    };
    can1Send_TaskHandle = osThreadNew(can1SendTask, NULL, &can1SendTaskHandle_attributes);

    const osThreadAttr_t can2SendTaskHandle_attributes = {
        .name = "can2Send_TaskHandle",
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityRealtime,
    };
    can2Send_TaskHandle = osThreadNew(can2SendTask, NULL, &can2SendTaskHandle_attributes);

    const osThreadAttr_t can3SendTaskHandle_attributes = {
        .name = "can3Send_TaskHandle",
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityRealtime,
    };
    can3Send_TaskHandle = osThreadNew(can3SendTask, NULL, &can3SendTaskHandle_attributes);

    const osThreadAttr_t chassisTaskHandle_attributes = {
        .name = "chassis_TaskHandle",
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityRealtime,
    };
    Chassis_TaskHandle = osThreadNew(chassisTask, NULL, &chassisTaskHandle_attributes);

    
}
