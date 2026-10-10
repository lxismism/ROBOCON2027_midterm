/**
 * @file Motor.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "Canbus.hpp"
#include "pid_controller.h"
#include "math_utils.hpp"

#define RAD_2_DEG            57.2957795f
#define DEG_2_RAD            0.01745329252f
#define RPM_2_DEG_PER_SEC    6.0f
#define RPM_2_RAD_PER_SEC    0.104719755f

class MotorBase{
public:
    MotorBase() = default;

    enum class PIDMode : uint8_t { NONE = 0, SPEED, DEGREE};

    void setMotorCmd(float cmd){
        if(cmd > max_cmd_){
            cmd = max_cmd_;
        }
        if(cmd < -max_cmd_){
            cmd = -max_cmd_;
        }
        cmd_ = cmd;
    }
    
    void setMotorDegSpeed(float deg_speed) { ref_deg_speed_ = deg_speed; }
    void setMotorDeg(float deg) { ref_deg_ = deg; }



    void pidDegUpdate(void) {
            ref_deg_speed_ = PID_Calculate(&deg_pid_, sum_deg_, ref_deg_);
    }

    void pidDegSpeedUpdate(void){
        cmd_ = PID_Calculate(&deg_speed_pid_, deg_speed_, ref_deg_speed_);
    }

    void pidUpdate(void){
        if(output_type_ == PIDMode::DEGREE ) pidDegUpdate();
        if(output_type_ != PIDMode::NONE) pidDegSpeedUpdate();
    }

    float getSingleDeg(void) const { return single_deg_; }
    float getSumDeg(void) const { return sum_deg_; }
    float getDegSpeed(void) const { return deg_speed_; }
    float getTorque(void) const {return torque_; }
    float getTemperature(void) const { return temperature_; }

protected:
    float cmd_{0.0f};
    float ref_deg_speed_{0.0f};
    float ref_deg_{0.0f};
    
    float max_cmd_{99999.0f};
    float reduction_ratio_{1.0f};

    float single_deg_{0.0f};    //转子在单圈内的绝对位置
    float sum_deg_{0.0f};       //电机输出轴累计的角度
    float deg_speed_{0.0f};     //电机输出轴角速度
    float torque_{0.0f};        //输出轴扭矩
    float temperature_{0.0f};

    PID_t deg_speed_pid_{};
    PID_t deg_pid_{};
    PIDMode output_type_{};
};


