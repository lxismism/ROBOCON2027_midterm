/**
 * @file pickhand_solution.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "math_utils.hpp"

#include "djiMotor.hpp"
#include "dmMotor.hpp"

#include "bool_device.hpp"

#include "topic_pool.h"

class PickHand{
public:
    PickHand(C620Motor* lift_motor, C620Motor* tripod_motor,
        C610Motor* extend_motor, DM4310Motor* wrist_motor,
        BoolDevice* pump, BoolDevice* valve) :
        lift_motor_(lift_motor), tripod_motor_(tripod_motor),
        extend_motor_(extend_motor), wrist_motor_(wrist_motor),
        pump_(pump), valve_(valve){}

    void run(const pub_pickhand_cmd &cmd){

        lift_motor_->setMotorDeg(cmd.lift_deg);
        tripod_motor_->setMotorDeg(cmd.tripod_deg);
        extend_motor_->setMotorDeg(cmd.extend_deg);
        
        if(cmd.flip){
            wrist_motor_->setMotorDeg(kflipdeg);
            wrist_motor_->setMotorDegSpeed(360.0f);
        } else {
            wrist_motor_->setMotorDeg(kflapdeg);
            wrist_motor_->setMotorDegSpeed(360.0f);
        }

        if(cmd.suction_on){
            pump_->On();
            valve_->On();
        } else {
            pump_->Off();
            valve_->Off();
        }
    }


private:
    static constexpr float kflipdeg = 90.0f;
    static constexpr float kflapdeg = 0.0f;

    C620Motor* lift_motor_{};
    C620Motor* tripod_motor_{};
    C610Motor* extend_motor_{};
    DM4310Motor* wrist_motor_{};
    BoolDevice* pump_{};
    BoolDevice* valve_{};

};