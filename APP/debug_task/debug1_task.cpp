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
#include "Remote_api.hpp"

osThreadId_t Debug1_TaskHandle;

// 实例与其他电机一样定义在 com_config.cpp，这里只设置目标。
extern C620Motor lifting_motor;
extern C620Motor spin_motor;
extern C610Motor elongate_motor;
extern DM4310Motor motor_dm;

// 遥控累加的位置目标，初始值为 0，单位为度。
float lifting_motor_target_deg = 0.0f;
float spin_motor_target_deg = 0.0f;
float elongate_motor_target_deg = 0.0f;
float motor_dm_target_deg = 0.0f;
float motor_dm_speed_deg_s = 60.0f;

// 按住按键时目标角度的变化速度，单位为度/秒，可分别调整。
float lifting_motor_adjust_deg_s = 360.0f;
float spin_motor_adjust_deg_s = 360.0f;
float elongate_motor_adjust_deg_s = 30.0f;
float motor_dm_adjust_deg_s = 30.0f;

static TypedTopicSubscriber<Remote::State>
    remote_data_sub(Remote::Remote_state_topic, 8U);
static Remote::State remote_data{};

static TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data", 8U);
static pub_imu_data imu_data{};
 int direction3508_1 ;
void debug1Task(void *argument)
{
    (void)argument;
    TickType_t currentTime = xTaskGetTickCount();
    TickType_t lastTime = currentTime;

    for(;;)
    {
        imu_data_sub.TryGet(&imu_data);

        const TickType_t now = xTaskGetTickCount();
        float dt = static_cast<float>(now - lastTime) /
                   static_cast<float>(configTICK_RATE_HZ);
        lastTime = now;
        // 调试暂停或调度延迟后，避免一次累加过大的角度。
        if(dt > 0.02f) dt = 0.02f;

        taskENTER_CRITICAL();
        // 取最新按键状态；没有新消息时沿用上一帧，不额外检查遥控超时。
        while(remote_data_sub.TryGet(&remote_data)){}

        // 直接按键控制，不使用 Sw1 和电机 ready 条件。
        // 同一对按键同时按下时，方向相减为 0，保持目标。
        direction3508_1 =
            static_cast<int>(Remote::is_pressed(remote_data, Remote::key::Lt)) -
            static_cast<int>(Remote::is_pressed(remote_data, Remote::key::Lb));
        const int direction3508_2 =
            static_cast<int>(Remote::is_pressed(remote_data, Remote::key::Rt)) -
            static_cast<int>(Remote::is_pressed(remote_data, Remote::key::Rb));
        const int direction2006 =
            static_cast<int>(Remote::is_pressed(remote_data, Remote::key::Up)) -
            static_cast<int>(Remote::is_pressed(remote_data, Remote::key::Down));
        const int direction_dm =
            static_cast<int>(Remote::is_pressed(remote_data, Remote::key::Right)) -
            static_cast<int>(Remote::is_pressed(remote_data, Remote::key::Left));

        if(direction3508_1 != 0){
            lifting_motor_target_deg += -direction3508_1 * lifting_motor_adjust_deg_s * dt;
        }
        if(direction3508_2 != 0){
            spin_motor_target_deg += -direction3508_2 * spin_motor_adjust_deg_s * dt;
        }
        if(direction2006 != 0){
            elongate_motor_target_deg += direction2006 * elongate_motor_adjust_deg_s * dt;
        }
        if(direction_dm != 0){
            motor_dm_target_deg += direction_dm * motor_dm_adjust_deg_s * dt;
        }

        lifting_motor.setMotorDeg(lifting_motor_target_deg);
        spin_motor.setMotorDeg(spin_motor_target_deg);
        elongate_motor.setMotorDeg(elongate_motor_target_deg);
        motor_dm.setMotorDeg(motor_dm_target_deg);
        motor_dm.setMotorDegSpeed(motor_dm_speed_deg_s);
        taskEXIT_CRITICAL();

        vTaskDelayUntil(&currentTime, 5);
    }

}
