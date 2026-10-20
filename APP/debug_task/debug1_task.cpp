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
 * @brief float -> 两位小数字符串，代替 newlib-nano 不支持的 %f
 *
 * 用法： snprintf(buf, n, "%s,%s", f2(a), f2(b))
 * 每次调用占用一个独立的静态缓冲区，所以一帧里最多写 24 个 f2()；
 * 只在本任务里用，不可重入。
 */
static const char *f2(float value)
{
    static constexpr uint8_t kPoolSize = 24U;
    static char pool[kPoolSize][16];
    static uint8_t index = 0U;

    char *buf = pool[index];
    index = static_cast<uint8_t>((index + 1U) % kPoolSize);

    const int scaled = static_cast<int>(std::fabs(value) * 100.0f + 0.5f);
    snprintf(buf, sizeof(pool[0]), "%s%d.%02d",
             (value < 0.0f) ? "-" : "",
             scaled / 100, scaled % 100);
    return buf;
}

void debug1Task(void *argument)
{
    (void)argument;
    TickType_t currentTime = xTaskGetTickCount();
    static char debug_buffer[256];

    for(;;)
    {
        imu_data_sub.TryGet(&imu_data);

        /* FireWater 格式：每行 "名字:值1,值2,..."，行尾 \n。
         * 要加减监测变量，只改下面的格式串和参数列表即可（值用 f2() 包一下）。 */
        int len = snprintf(debug_buffer, sizeof(debug_buffer),
            "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
            //"%s,%s,%s,%s,%s,%s,%s,%s\n"
            //"%s\n",
            
            // 驱动轮 
            f2(chassis_drivemotors[0]->getRefDegSpeed()), //目标值
            f2(chassis_drivemotors[0]->getDegSpeed()),    //实际值
            f2(chassis_drivemotors[1]->getRefDegSpeed()),
            f2(chassis_drivemotors[1]->getDegSpeed()),
            f2(chassis_drivemotors[2]->getRefDegSpeed()),
            f2(chassis_drivemotors[2]->getDegSpeed()),
            f2(chassis_drivemotors[3]->getRefDegSpeed()),
            f2(chassis_drivemotors[3]->getDegSpeed()),

            
            // 舵向轮 
            f2(chassis_dirmotors[0]->getRefDeg()),  //目标值
            f2(chassis_dirmotors[0]->getSumDeg()),  //实际值
            f2(chassis_dirmotors[1]->getRefDeg()),
            f2(chassis_dirmotors[1]->getSumDeg()),
            f2(chassis_dirmotors[2]->getRefDeg()),
            f2(chassis_dirmotors[2]->getSumDeg()),
            f2(chassis_dirmotors[3]->getRefDeg()),
            f2(chassis_dirmotors[3]->getSumDeg()),
            

            // IMU：yaw 
            f2(imu_data.yaw_deg));

        /* snprintf 返回值可能大于缓冲区长度，截断一下再发 */
        if(len > static_cast<int>(sizeof(debug_buffer)) - 1){
            len = static_cast<int>(sizeof(debug_buffer)) - 1;
        }
        if(len > 0){
            uart4_port.writeDma(reinterpret_cast<const uint8_t*>(debug_buffer),
                                static_cast<size_t>(len));
        }

        vTaskDelayUntil(&currentTime, 20);   /* 50Hz：115200 下约 8.6KB/s */
    }

}
