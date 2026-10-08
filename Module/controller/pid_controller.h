/**
 * @file pid_controller.h
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

typedef enum pid_Inprovement_e{
    NONE = 0x00,
    OutputFilter = 0x10,
} pid_Inprovement_e;

typedef struct{
    float Kp;
    float Ki;
    float Kd;

    float Ref;
    float Measure;
    float Err;
    float Last_Err;

    float ITerm;
    float Pout, Iout, Dout;
    float Output;
    float Last_Output;

    float IntegralLimit;
    float Maxout;

    uint32_t DWT_CNT;
    float dt;

    float Output_LPF_RC;
    uint16_t Improve;
} PID_t;

void PID_Init(PID_t *pid);
float PID_Calculate(PID_t *pid, float measure, float ref);

#ifdef __cplusplus
}
#endif


#endif