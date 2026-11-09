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
#include "htDebug.hpp"

#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "chassis_solution.hpp"
#include "chassis_task.h"
#include "math_utils.hpp"

#include "usart.h"             /* huart3：调试串口（115200） */
#include "fdcan.h"             /* hfdcan3：读 PSR/ECR 判断物理层 */
#include <stdio.h>
#include <stdarg.h>
osThreadId_t Debug1_TaskHandle;

static TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data", 8U);
static pub_imu_data imu_data{};

extern HightorqueMotor high_torque_motor;
extern CanTraceDevice fdcan3_trace;
extern volatile float   ht_target_deg_s;
extern volatile float   ht_target_deg;
extern volatile uint8_t ht_cmd_ack;

/* ==================== 高擎电机测试参数（只改这里） ====================
 * 8 字节经典帧，走 fdcan3，不影响达妙 / 大疆电机。
 *   m<度/秒>  改速度      p<度>  位置模式（相对上电位置）      x  停转
 */
#define HT_BOOT_DEG_PER_SEC     240.0f   /* 上电默认目标速度，0 = 上电不转 */
#define HT_ACCEL_LIMIT_DEG_S2   1800.0f  /* 加速度限幅，0→240 度/秒约 0.13s */
#define HT_PRINT_PERIOD_MS      500U     /* 串口打印周期 */
#define HT_TRACE_PERIOD         (5U)     /* 每 5 次打印跟一次总线快照（2.5s） */

/* UART3 直接发送，不依赖 printf 重定向（本工程 _write 指向 huart4，
   而串口助手接的是 UART3，波特率 115200）。 */
static void htLog(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)sizeof(buf) - 1) {
            n = (int)sizeof(buf) - 1;
        }
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 50);
    }
}

/* hex 打印：8 字节 → "07 07 00 80 6B 0A 00 80"（23 字符 + '\0'） */
static void hexStr(char *out, const uint8_t *d, uint8_t n)
{
    static const char kHex[] = "0123456789ABCDEF";
    uint8_t k = 0;
    for (uint8_t i = 0; i < n; ++i) {
        out[k++] = kHex[(d[i] >> 4) & 0x0FU];
        out[k++] = kHex[d[i] & 0x0FU];
        out[k++] = ' ';
    }
    if (k > 0U) {
        k--;
    }
    out[k] = '\0';
}

static const char *lecToStr(uint32_t lec)
{
    switch (lec) {
        case 0:  return "NONE";
        case 1:  return "STUFF";
        case 2:  return "FORM";
        case 3:  return "ACK";   /* 没人应答：只有自己在线 / 波特率不对 / 没接终端电阻 */
        case 4:  return "BIT1";
        case 5:  return "BIT0";
        case 6:  return "CRC";
        default: return "OTHER";
    }
}

static void printFdcan3Status(void)
{
    const uint32_t psr = hfdcan3.Instance->PSR;
    const uint32_t ecr = hfdcan3.Instance->ECR;
    const uint32_t lec = psr & 0x07U;
    const uint32_t tec = ecr & 0xFFU;
    const uint32_t rec = (ecr >> 8) & 0x7FU;

    htLog("[HT] C3 LEC=%s(%lu) TEC=%lu REC=%lu n=%lu\r\n",
          lecToStr(lec), (unsigned long)lec,
          (unsigned long)tec, (unsigned long)rec,
          (unsigned long)fdcan3_trace.count());
}

/* 最近 3 帧总线内容：确认达妙/大疆还在、确认电机回包 ID */
static void printFdcan3Trace(void)
{
    const uint32_t n = fdcan3_trace.count();
    if (n == 0U) {
        return;
    }

    const uint32_t depth = (n < 3U) ? n : 3U;
    for (uint32_t i = 0; i < depth; ++i) {
        const CanTraceDevice::Slot &s = fdcan3_trace.get(i);
        char hex[3U * 8U + 1U];
        hexStr(hex, s.data, s.len);
        htLog("[HT] BUS id=%04lX %c %uB | %s\r\n",
              (unsigned long)s.id, s.ext ? 'E' : 'S', (unsigned)s.len, hex);
    }
}

void debug1Task(void *argument)
{
    TickType_t currentTime = xTaskGetTickCount();
    (void)argument;

    /* 模式：1 = 速度，2 = 位置（由串口命令切换） */
    uint8_t want_mode = 1U;
    uint32_t print_cnt = 0;
    uint32_t last_tx = 0;
    uint32_t last_rx = 0;

    ht_target_deg_s = HT_BOOT_DEG_PER_SEC;
    high_torque_motor.setAccelLimit(HT_ACCEL_LIMIT_DEG_S2);
    high_torque_motor.setTargetSpeed(ht_target_deg_s);

    htLog("\r\n[HT] boot: classic8 id=1 proto=07-07 target=%ld deg/s accel=%ld\r\n",
          (long)HT_BOOT_DEG_PER_SEC, (long)HT_ACCEL_LIMIT_DEG_S2);
    htLog("[HT] cmd: m<deg/s> speed | p<deg> position | x stop\r\n");

    for(;;)
    {
        imu_data_sub.TryGet(&imu_data);

        /* 串口命令回执 */
        const uint8_t ack = ht_cmd_ack;
        if (ack != 0U) {
            ht_cmd_ack = 0U;
            if (ack == 1U) {
                want_mode = 1U;
                htLog("[HT] cmd ok -> speed %ld deg/s\r\n", (long)ht_target_deg_s);
            } else if (ack == 2U) {
                want_mode = 2U;
                htLog("[HT] cmd ok -> position %ld deg (need feedback)\r\n",
                      (long)ht_target_deg);
            } else if (ack == 3U) {
                want_mode = 1U;
                htLog("[HT] cmd ok -> stop\r\n");
            } else {
                htLog("[HT] cmd bad, use m240 / p30 / x\r\n");
            }
        }

        /* 唯一的下发点：每 5ms 刷新一次目标，斜坡在 update() 里按真实时间做 */
        if (want_mode == 1U) {
            high_torque_motor.setTargetSpeed(ht_target_deg_s);
        } else {
            high_torque_motor.setTargetPosition(ht_target_deg);
        }

        if (++print_cnt >= (HT_PRINT_PERIOD_MS / 5U)) {
            print_cnt = 0U;

            /* 1) 发出去的最后一帧 */
            {
                char hex[3U * 8U + 1U];
                hexStr(hex, high_torque_motor.getLastTxData(), 8U);
                htLog("[HT] T id=%04lX %c | %s\r\n",
                      (unsigned long)high_torque_motor.getLastTxId(),
                      high_torque_motor.isLastTxExt() ? 'E' : 'S', hex);
            }

            /* 2) 收到的最后一帧 */
            if (high_torque_motor.getRxFrames() == 0U) {
                htLog("[HT] R <NO RX FRAME YET>\r\n");
            } else {
                char hex[3U * 8U + 1U];
                hexStr(hex, high_torque_motor.getLastRxData(),
                       high_torque_motor.getLastRxLen());
                htLog("[HT] R id=%04lX %c | %s\r\n",
                      (unsigned long)high_torque_motor.getLastRxId(),
                      high_torque_motor.isLastRxExt() ? 'E' : 'S', hex);
            }

            /* 3) 电机状态（全部整数，避免 %f 打空） */
            {
                const uint32_t tx = high_torque_motor.getTxFrames();
                const uint32_t rx = high_torque_motor.getRxFrames();
                htLog("[HT] %s sync=%d tx=%lu(+%lu) rx=%lu(+%lu) fb=%lu fail=%lu\r\n",
                      high_torque_motor.getStateName(),
                      (int)high_torque_motor.isSynced(),
                      (unsigned long)tx, (unsigned long)(tx - last_tx),
                      (unsigned long)rx, (unsigned long)(rx - last_rx),
                      (unsigned long)high_torque_motor.getFbFrames(),
                      (unsigned long)high_torque_motor.getTxFail());
                last_tx = tx;
                last_rx = rx;

                htLog("[HT] tgt=%ld cmd=%ld fb_mdeg=%ld fb_mdegs=%ld tqe=%d\r\n",
                      (long)high_torque_motor.getTargetDegSpeed(),
                      (long)high_torque_motor.getCmdDegSpeed(),
                      (long)(high_torque_motor.getFbPosDeg() * 1000.0f),
                      (long)(high_torque_motor.getFbVelDegS() * 1000.0f),
                      (int)high_torque_motor.getFbTqeRaw());
            }

            /* 4) 总线错误计数 */
            printFdcan3Status();

            /* 5) 每 2.5s 跟一次总线快照 */
            static uint8_t trace_div = 0;
            if (++trace_div >= HT_TRACE_PERIOD) {
                trace_div = 0;
                printFdcan3Trace();
            }
        }

        vTaskDelayUntil(&currentTime, 5);
    }

}