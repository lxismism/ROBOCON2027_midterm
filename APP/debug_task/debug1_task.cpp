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
#include "hightorqueMotor.hpp"

#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "chassis_solution.hpp"
#include "chassis_task.h"
#include "math_utils.hpp"

#include "usart.h"             /* huart3：调试串口 */
#include <stdio.h>
#include <stdarg.h>
osThreadId_t Debug1_TaskHandle;

static TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data", 8U);
static pub_imu_data imu_data{};

extern HightorqueMotor high_torque_motor;

/* ==================== 高擎电机测试参数（只改这里） ====================
 * 帧格式：07 20 <pos:2> <vel:2> <tqe:2>，8 字节经典 CAN，走 fdcan3。
 *   位置 int16：1 count = 0.0001 圈 → 1 圈 = 10000，量程 ±3.27 圈
 *   速度 int16：1 count = 0.00025 转/秒 → 1 转/秒 = 4000
 *
 * HT_TARGET_DEG 是「相对上电位置的偏移」，不是绝对角度：
 * 上电后先等一帧反馈锁存当前位置，再把指令从该位置斜坡推过去，
 * 这样即使不知道编码器零点也不会出现大跳变。
 */
#define HT_ENABLE              1          /* 0 = 完全关闭高擎测试，不发任何帧 */
#define HT_TARGET_DEG          30.0f      /* 目标偏移（输出轴角度），正负均可 */
#define HT_RAMP_DEG_PER_SEC    60.0f      /* 指令斜坡，越大动作越猛 */
#define HT_VEL_LIMIT_DEG_S     0.0f       /* >0 写 0x21 速度限幅；0 = 写 NaN 交给固件 */
#define HT_USE_MIT             0          /* 1 = 运控(MIT)模式，会先下发 Kp/Kd；0 = 纯位置模式 */
#define HT_MIT_KP              5.0f       /* 仅 MIT 模式生效 */
#define HT_MIT_KD              0.1f       /* 仅 MIT 模式生效 */
#define HT_TORQUE_LIMIT_NM     0.0f       /* >0 下发 0x25 最大扭矩；0 = 不下发 */
#define HT_PRINT_PERIOD_MS     200U       /* 串口打印周期 */
/* 收不到反馈时是否强行运行：0 = 不动并打印 NOFB（安全，默认）；
 * 1 = 把当前位置当 0 强行跑（会有一次未知大小的初始跳变，仅台架无负载时用） */
#define HT_BLIND_START         0

/* UART3 直接发送，不依赖 printf 重定向（本工程 _write 指向 huart4）。
 * 波特率沿用 usart.c 里的设置（当前 420000），串口助手请按此设置。*/
static void htLog(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)sizeof(buf) - 1) {
            n = (int)sizeof(buf) - 1;
        }
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 20);
    }
}
void debug1Task(void *argument)
{
    TickType_t currentTime = xTaskGetTickCount();

#if HT_ENABLE
    high_torque_motor.setRampDegPerSec(HT_RAMP_DEG_PER_SEC);
    high_torque_motor.setVelLimitDegS(HT_VEL_LIMIT_DEG_S);
    high_torque_motor.setTorqueLimitNm(HT_TORQUE_LIMIT_NM);
    high_torque_motor.setAllowBlindStart(HT_BLIND_START);
#if HT_USE_MIT
    high_torque_motor.setMitGains(HT_MIT_KP, HT_MIT_KD);
    high_torque_motor.setTargetPositionMit(HT_TARGET_DEG);
#else
    high_torque_motor.setTargetPosition(HT_TARGET_DEG);
#endif
    htLog("\r\n[HT] start classic8 id=%lu mode=%d mit=%d target=%.1f ramp=%.0f\r\n",
          (unsigned long)high_torque_motor.getTxID(),
          (int)high_torque_motor.getMode(),
          (int)HT_USE_MIT,
          (double)HT_TARGET_DEG,
          (double)HT_RAMP_DEG_PER_SEC);
#endif

    uint32_t print_cnt = 0;

    for(;;)
    {
        imu_data_sub.TryGet(&imu_data);

#if HT_ENABLE
        if (++print_cnt >= (HT_PRINT_PERIOD_MS / 5U)) {
            print_cnt = 0;
            htLog("[HT] %s sync=%d%s tx=%lu rx=%lu fb=%lu pos=%.2f vel=%.2f tgt=%.2f err=%.2f\r\n",
                  high_torque_motor.getStateName(),
                  (int)high_torque_motor.isSynced(),
                  high_torque_motor.isSyncTimeout() ? " TO!" : "",
                  (unsigned long)high_torque_motor.getTxFrames(),
                  (unsigned long)high_torque_motor.getRxFrames(),
                  (unsigned long)high_torque_motor.getFbFrames(),
                  (double)high_torque_motor.getFbPosDeg(),
                  (double)high_torque_motor.getFbVelDeg(),
                  (double)high_torque_motor.getTargetDeg(),
                  (double)high_torque_motor.getErrDeg());
        }
#endif

        vTaskDelayUntil(&currentTime, 5);
    }

}