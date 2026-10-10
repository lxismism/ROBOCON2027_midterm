/**
 * @file control_task.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "control_task.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include "math_utils.hpp"

#include "topic_pool.h"
#include "topics.hpp"
#include "Remote_api.hpp"

osThreadId_t control_TaskHandle;

/*订阅遥控信息*/
static TypedTopicSubscriber<Remote::State> 
    remote_data_sub(Remote::Remote_state_topic, 8U);
static Remote::State sub_remote_data{};

/*发布底盘命令(旋量)*/
static TypedTopicPublisher<pub_chassis_cmd> chassis_cmd_pub("chassis_cmd");
static pub_chassis_cmd chassis_cmd{};


void Remote_Data_Process()
{
    static constexpr float kMaxCh = 2048.0f;
    static constexpr float kMaxSpeed = 1.5f;

    const float left_x = static_cast<float>(Remote::get_channel(sub_remote_data, Remote::Channel::LEFT_X));
    const float left_y = static_cast<float>(Remote::get_channel(sub_remote_data, Remote::Channel::LEFT_Y));
    const float right_x = static_cast<float>(Remote::get_channel(sub_remote_data, Remote::Channel::RIGHT_X));
    const float right_y = static_cast<float>(Remote::get_channel(sub_remote_data, Remote::Channel::RIGHT_Y));
    
    if(std::fabs(left_x) < 125.0f){
        chassis_cmd.linear_x_ = 0.0f;
    } else{
        chassis_cmd.linear_x_ = math_utils::Map(left_x, -kMaxCh, 
            kMaxCh, -kMaxSpeed, kMaxSpeed); 
    }

    if(std::fabs(left_y) < 125.0f){
        chassis_cmd.linear_y_ = 0.0f;
    } else{
        chassis_cmd.linear_y_ = math_utils::Map(left_y, -kMaxCh, 
            kMaxCh, -kMaxSpeed, kMaxSpeed); 
    }

    if(std::fabs(right_x) < 125.0f){
        chassis_cmd.omega_ = 0.0f;
    } else{
        chassis_cmd.omega_ = math_utils::Map(right_x, -kMaxCh, 
            kMaxCh, -kPI, kPI); 
    }
    
}

void controlTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){

        if(remote_data_sub.TryGet(&sub_remote_data)){
            Remote_Data_Process();
            chassis_cmd_pub.Publish(chassis_cmd);
        }

        vTaskDelayUntil(&current, 5);
    }

}