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

#include <array>
#include <cmath>
#include <cstdio>

osThreadId_t Debug1_TaskHandle;

static TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data", 8U);
static pub_imu_data imu_data{};

extern UartPort uart4_port;
extern std::array<MotorBase*, 4> chassis_drivemotors;
extern std::array<MotorBase*, 4> chassis_dirmotors;

/**
 * @brief 定点两位小数，代替 newlib-nano 不支持的 %f
 * @return 写入后的位置
 */
static int appendFixed2(char *buf, size_t size, int pos, float value)
{
    const int scaled = static_cast<int>(std::fabs(value) * 100.0f + 0.5f);
    return pos + snprintf(buf + pos, size - static_cast<size_t>(pos),
                          "%s%d.%02d",
                          (value < 0.0f) ? "-" : "",
                          scaled / 100, scaled % 100);
}

void debug1Task(void *argument)
{
    (void)argument;
    TickType_t currentTime = xTaskGetTickCount();
    static char debug_buffer[256];

    for(;;)
    {
        imu_data_sub.TryGet(&imu_data);

        int pos = 0;

        /* 驱动轮：4 个电机的 [目标速度, 实际速度] */
        pos += snprintf(debug_buffer + pos, sizeof(debug_buffer) - pos, "drive:");
        for(uint8_t i = 0; i < 4U; ++i){
            pos += snprintf(debug_buffer + pos, sizeof(debug_buffer) - pos, ",");
            pos += appendFixed2(debug_buffer, sizeof(debug_buffer), pos,
                                chassis_drivemotors[i]->getRefDegSpeed());
            pos += snprintf(debug_buffer + pos, sizeof(debug_buffer) - pos, ",");
            pos += appendFixed2(debug_buffer, sizeof(debug_buffer), pos,
                                chassis_drivemotors[i]->getDegSpeed());
        }

        /* 舵向轮：4 个电机的 [目标角度, 实际角度] */
        pos += snprintf(debug_buffer + pos, sizeof(debug_buffer) - pos, "\nsteer:");
        for(uint8_t i = 0; i < 4U; ++i){
            pos += snprintf(debug_buffer + pos, sizeof(debug_buffer) - pos, ",");
            pos += appendFixed2(debug_buffer, sizeof(debug_buffer), pos,
                                chassis_dirmotors[i]->getRefDeg());
            pos += snprintf(debug_buffer + pos, sizeof(debug_buffer) - pos, ",");
            pos += appendFixed2(debug_buffer, sizeof(debug_buffer), pos,
                                chassis_dirmotors[i]->getSumDeg());
        }

        /* IMU：yaw */
        pos += snprintf(debug_buffer + pos, sizeof(debug_buffer) - pos, "\nimu:");
        pos += appendFixed2(debug_buffer, sizeof(debug_buffer), pos, imu_data.yaw_deg);
        pos += snprintf(debug_buffer + pos, sizeof(debug_buffer) - pos, "\n");

        uart4_port.writeDma(reinterpret_cast<const uint8_t*>(debug_buffer), pos);

        vTaskDelayUntil(&currentTime, 20);   /* 50Hz：115200 下约 8.6KB/s */
    }

}
