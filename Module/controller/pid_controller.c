/**
 * @file pid_controller.c
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "pid_controller.h"
#include "bsp_dwt.h"

#define MAX_DT 0.02f

static void f_Iout_Clamp(PID_t *pid);
static void f_Derivate_On_Measurement(PID_t *pid);
static void f_Output_Filter(PID_t *pid);
static void f_Output_Clamp(PID_t *pid);


float PID_Calculate(PID_t *pid, float measure, float ref)
{
    const uint8_t inc_mode = (pid->Improve & INCREATEMENT_OF_OUT) ? 1 : 0;

    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;
    
    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT);

    if(pid->dt > MAX_DT){
        
        pid->Earlier_Measure = pid->Measure;
        pid->Last_Measure = pid->Measure;
            
        if(inc_mode){
            pid->Earlier_Err = pid->Last_Err;
        }    
        else{
            pid->Earlier_Err = pid->Err;
            pid->Last_Err = pid->Err;
        }
            
        return pid->Output;
    }   //采样周期过长，系统超时，直接抛弃这一帧计算

    if(pid->dt <= 1e-6f) pid->dt = 1e-6f;

    const uint8_t in_deadband = (pid->Err > -pid->DeadBand && pid->Err < pid->DeadBand) ? 1 : 0;

    if(in_deadband){
        if(inc_mode){
            pid->Pout = 0.0f;
            pid->ITerm = 0.0f;
            pid->Dout = 0.0f;
        } else {
            pid->Err = 0.0f;
            pid->Pout = 0.0f;
            pid->ITerm = 0.0f;
            pid->Dout = 0.0f;
        }
    }
    else if(inc_mode){
        pid->Pout = pid->Kp * (pid->Err - pid->Last_Err);
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err + pid->Earlier_Err - 2.0f * pid->Last_Err) / pid->dt;
    }
    else{
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
    }

    pid->Iout += pid->ITerm;
    f_Iout_Clamp(pid);

    if(pid->Improve & Derivative_On_Measurement)
        f_Derivate_On_Measurement(pid);

    pid->Output = (inc_mode) ?
        (pid->Last_Output + pid->Pout + pid->ITerm + pid->Dout) :
        (pid->Pout + pid->Iout + pid->Dout);
    
    if(pid->Improve & OutputFilter)
        f_Output_Filter(pid);

    f_Output_Clamp(pid);

    //状态更新
    pid->Earlier_Err = pid->Last_Err;
    pid->Last_Err = pid->Err;
    pid->Earlier_Measure = pid->Last_Measure;
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;

    return pid->Output;
    
}

static void f_Iout_Clamp(PID_t *pid){
    pid->Iout = (pid->Iout >= pid->IntegralLimit)? pid->IntegralLimit :
        ((pid->Iout <= -pid->IntegralLimit)? -pid->IntegralLimit : pid->Iout);
}

static void f_Derivate_On_Measurement(PID_t *pid){
    if(pid->Improve & INCREATEMENT_OF_OUT)
        pid->Dout = pid->Kd * (2.0f * pid->Last_Measure - pid->Measure - pid->Earlier_Measure) / pid->dt;
    else
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;

}

static void f_Output_Filter(PID_t *pid){
    pid->Output = (pid->Output * pid->dt + pid->Output_LPF_RC * pid->Last_Output) /
                 (pid->Output_LPF_RC + pid->dt);
}

static void f_Output_Clamp(PID_t *pid){
    pid->Output = (pid->Output >= pid->Maxout)? pid->Maxout :
        ((pid->Output <= -pid->Maxout)? -pid->Maxout : pid->Output);
}

void PID_Reset(PID_t *pid){
    pid->Ref = 0.0f;
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Earlier_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Earlier_Err = 0.0f;

    pid->ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;

    pid->DWT_CNT = 0;
    pid->dt = 0.0f;
}

