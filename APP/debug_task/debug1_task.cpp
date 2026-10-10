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
#include "MotorBase.hpp"
#include "djiMotor.hpp"
#include "dmMotor.hpp"
#include "vescMotor.hpp"

#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "chassis_solution.hpp"
#include "chassis_task.h"
#include "math_utils.hpp"
#include "Remote_receiver.hpp"
#include "Remote_api.hpp"
osThreadId_t Debug1_TaskHandle;

extern C620Motor chassis_dirmotor1;
// 启动阶段注册，必须早于 subsQueueInit()。
static TypedTopicSubscriber<Remote::State>
    remote_subscriber(Remote::Remote_state_topic, 8U);

Remote::State remote_debug_state{};
float ch1,ch2,ch3,ch4;
bool up_pressed,down_pressed;
bool sw1_up;
uint8_t left_pressed;
void debug1Task(void *argument)
{
    TickType_t currentTime = xTaskGetTickCount();
    

    for(;;)
    {
        Remote::State message{};
        for (;;) {
            bool received = false;

            taskENTER_CRITICAL();
            received = remote_subscriber.TryGet(&message);
            taskEXIT_CRITICAL();

            if (!received) {
                break;
            }

            remote_debug_state = message;
            ch1 = Remote::get_channel(remote_debug_state, Remote::Channel::CH1);
            ch2 = Remote::get_channel(remote_debug_state, Remote::Channel::CH2);
            ch3 = Remote::get_channel(remote_debug_state, Remote::Channel::CH3);
            ch4 = Remote::get_channel(remote_debug_state, Remote::Channel::CH4);
            up_pressed = Remote::is_pressed(remote_debug_state, Remote::key::Up);
            down_pressed = Remote::is_pressed(remote_debug_state, Remote::key::Down);
            sw1_up = Remote::is_pressed(remote_debug_state, Remote::key::Sw1);
            left_pressed = Remote::is_pressed(remote_debug_state, Remote::key::Left);
        }

        // 没有新消息时，也检查本地状态是否已经过期。
        if (!Remote::is_fresh(
                remote_debug_state,
                HAL_GetTick()))
        {
            remote_debug_state.online = 0U;
        }
        vTaskDelayUntil(&currentTime, 1);
    }

}