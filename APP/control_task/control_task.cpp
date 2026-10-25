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

/*发布吸取手命令*/
static TypedTopicPublisher<pub_pickhand_cmd> pickhand_cmd_pub("pickhand_cmd");
static pub_pickhand_cmd pickhand_cmd{};

/*发布武器手命令*/
static TypedTopicPublisher<pub_weaponhand_cmd> weaponhand_cmd_pub("weaponhand_cmd");
static pub_weaponhand_cmd weaponhand_cmd{};

void Remote_Data_Process()
{
    static constexpr float kMaxCh = 2048.0f;
    static constexpr float kMaxSpeed = 1.0f;

    const float left_x = static_cast<float>(Remote::get_channel(sub_remote_data, Remote::Channel::LEFT_X));
    const float left_y = static_cast<float>(Remote::get_channel(sub_remote_data, Remote::Channel::LEFT_Y));
    const float right_x = static_cast<float>(Remote::get_channel(sub_remote_data, Remote::Channel::RIGHT_X));
    const float right_y = static_cast<float>(Remote::get_channel(sub_remote_data, Remote::Channel::RIGHT_Y));
    
    if(std::fabs(left_x) < 850.0f){
        chassis_cmd.linear_x_ = 0.0f;
    } else{
        chassis_cmd.linear_x_ = math_utils::Map(left_x, -kMaxCh, 
            kMaxCh, -kMaxSpeed, kMaxSpeed); 
    }

    if(std::fabs(left_y) < 850.0f){
        chassis_cmd.linear_y_ = 0.0f;
    } else{
        chassis_cmd.linear_y_ = math_utils::Map(left_y, -kMaxCh, 
            kMaxCh, -kMaxSpeed, kMaxSpeed); 
    }

    if(std::fabs(right_x) < 850.0f){
        chassis_cmd.omega_ = 0.0f;
    } else{
        chassis_cmd.omega_ = -math_utils::Map(right_x, -kMaxCh, 
            kMaxCh, -kPI / 2.0f, kPI / 2.0f);   //这个负号是根据实际情况定的
            
    }

    if(Remote::SW1 & sub_remote_data.key){
        pickhand_cmd.lift_deg += 40.0f;
        pickhand_cmd.lift_deg = math_utils::Clamp(pickhand_cmd.lift_deg, 0.0f, 2220.0f);
    } 
    if(Remote::SW4 & sub_remote_data.key){
        pickhand_cmd.lift_deg -= 40.0f;
        pickhand_cmd.lift_deg = math_utils::Clamp(pickhand_cmd.lift_deg, 0.00f, 2220.0f);
    }   //SW1,SW4吸取手上升，下降

    if(Remote::SW2 & sub_remote_data.key){
        pickhand_cmd.tripod_deg += 25.0f;
        pickhand_cmd.tripod_deg = math_utils::Clamp(pickhand_cmd.tripod_deg, -375.0f, 375.0f);
    }   

    if(Remote::SW3 & sub_remote_data.key){
        pickhand_cmd.tripod_deg -= 25.0f;
        pickhand_cmd.tripod_deg = math_utils::Clamp(pickhand_cmd.tripod_deg, -375.0f, 375.0f);

    }   //SW2,SW3云台逆，顺转

    pickhand_cmd.flip = ((Remote::SW9 & sub_remote_data.key) != 0);
    pickhand_cmd.suction_on = ((Remote::SW11 & sub_remote_data.key) != 0);

    if(Remote::SWRB & sub_remote_data.key){
        pickhand_cmd.extend_deg -= 12.0f;
        pickhand_cmd.extend_deg = math_utils::Clamp(pickhand_cmd.extend_deg, -335.0f, 0.0f);
    }

    if(Remote::SWR & sub_remote_data.key){
        pickhand_cmd.extend_deg += 12.0f;
        pickhand_cmd.extend_deg = math_utils::Clamp(pickhand_cmd.extend_deg, -335.0f, 0.0f);

    }

    if(Remote::SWLB & sub_remote_data.key){
        weaponhand_cmd.lift_deg += 12.0f;
        weaponhand_cmd.lift_deg = math_utils::Clamp(weaponhand_cmd.lift_deg, -900.0f, 0.0f);

    }

    if(Remote::SWL & sub_remote_data.key){
        weaponhand_cmd.lift_deg -= 12.0f;
        weaponhand_cmd.lift_deg = math_utils::Clamp(weaponhand_cmd.lift_deg, -900.0f, 0.0f);
    }

    // if(Remote::SWLB & sub_remote_data.key){
    //     weaponhand_cmd.pick_deg += 5.1f;
    //     weaponhand_cmd.pick_deg = math_utils::Clamp(weaponhand_cmd.pick_deg, -900.0f, 0.0f);

    // }

    // if(Remote::SWL & sub_remote_data.key){
    //     weaponhand_cmd.pick_deg -= 5.1f;
    //     weaponhand_cmd.pick_deg = math_utils::Clamp(weaponhand_cmd.pick_deg, -900.0f, 0.0f);
    // }

    if(Remote::SW14 & sub_remote_data.key){
        weaponhand_cmd.pick = true;
    } else {
        weaponhand_cmd.pick = false;
    }
        
}

void controlTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){

        if(remote_data_sub.TryGet(&sub_remote_data)){
            Remote_Data_Process();
            pickhand_cmd_pub.Publish(pickhand_cmd);
            chassis_cmd_pub.Publish(chassis_cmd);
            weaponhand_cmd_pub.Publish(weaponhand_cmd);
        }

        vTaskDelayUntil(&current, 1);
    }

}