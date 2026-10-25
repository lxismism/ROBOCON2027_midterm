/**
 * @file weaponhand_solution.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-23
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "math_utils.hpp"

#include "djiMotor.hpp"
#include "topic_pool.h"

class Weaponhand{
public:
    Weaponhand(C620Motor* lift_motor, C610Motor* pick_motor) : 
        lift_motor_(lift_motor), pick_motor_(pick_motor){}

    void run(pub_weaponhand_cmd &weaponhand_cmd){


        lift_motor_->setMotorDeg(weaponhand_cmd.lift_deg, 360.0f);

        if(weaponhand_cmd.pick){
            pick_motor_->setMotorDeg(kPickDeg, 360.0f);
        } else {
            pick_motor_->setMotorDeg(0.0f, 360.0f);
        }
    }

private:
    static constexpr float kPickDeg = -44.0f;


    C620Motor* lift_motor_{};
    C610Motor* pick_motor_{};

};