/**
 * @file chassis_solution.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once
#include "Motor.hpp"
#include "topic_pool.h"
#include <cmath>
#include <array>

static constexpr float kPI = 3.1415926535f;
struct Velocity_t{
    float dir;
    float speed;
};

class SteerWheel{
    /*
        1    0   

        2    3
    */

public:
    SteerWheel() = default;

    void init(MotorBase* dirmotor, MotorBase* drivemotor){
        dirmotor_ = dirmotor;
        drivemotor_ = drivemotor;
    }

    void setVelocity(Velocity_t target_velocity){

        updateWheelState();

        float deg_output = 0.0f;        //由于电机位置环的输入是累计的绝对角度，所以此处也必须用舵轮的累计角度计算，输入和输出必须匹配
        float degspeed_output = 0.0f;
        float diff = target_velocity.dir - current_velocity_.dir;


        if(diff > 90.0f){
            if(diff > 270.0f){
                target_velocity.dir -= 360.0f;

            } else{
                target_velocity.dir -= 180.0f;
                target_velocity.speed = -target_velocity.speed;
            }
        }
        else if(diff < -90.0f){
            if(diff < -270.0f){
                target_velocity.dir += 360.0f;
                
            } else{
                target_velocity.dir += 180.0f;
                target_velocity.speed = -target_velocity.speed;
            }
        }
        diff = target_velocity.dir - current_velocity_.dir;
        

        deg_output = (wheel_sum_deg_ + diff) * kWheelDirReduction;
        degspeed_output = (target_velocity.speed) * RAD_2_DEG * kWheelDriveReduction / kWheelRadmeter; 

        dirmotor_->setMotorDeg(deg_output);
        drivemotor_->setMotorDegSpeed(degspeed_output);
    }
    


private:
    static constexpr float kWheelDiameter = 0.104f;     //轮径
    static constexpr float kWheelRadmeter = kWheelDiameter / 2.0f;      //轮半径
    static constexpr float kWheelCirmeter = kPI * kWheelDiameter;        //轮周
    static constexpr float kWheelDriveReduction = 1.0f;     //电机输出轴->驱动轮的减速比
    static constexpr float kWheelDirReduction = 1.0f;       //电机输出轴->舵向轮的减速比

    MotorBase* dirmotor_{};
    MotorBase* drivemotor_{};

    Velocity_t current_velocity_{};
    Velocity_t last_velocity_{};
    float wheel_sum_deg_{0.0f};

    void updateWheelState(){

        wheel_sum_deg_ = dirmotor_->getSumDeg() / kWheelDirReduction;
        
        current_velocity_.dir = math_utils::WrapAngle(wheel_sum_deg_,
            -180.0f, 180.0f);
        current_velocity_.speed = drivemotor_->getDegSpeed() * DEG_2_RAD * kWheelRadmeter / kWheelDriveReduction;
        
    }

};

class SteerChassis{
public:
    static constexpr uint8_t kWheelNum = 4U;        //轮数

    SteerChassis(const std::array<MotorBase* , kWheelNum> &dirmotors,
                 const std::array<MotorBase* , kWheelNum> &drivemotors){
        for(uint8_t i = 0; i < kWheelNum; ++i){
            steerwheels_[i].init(dirmotors[i], drivemotors[i]);
        }
    }

    void run(const pub_chassis_cmd &cmd){
        const float vx = cmd.linear_x_;
        const float vy = cmd.linear_y_;
        const float omega = cmd.omega_;

        velocitys_[0].speed = std::sqrt((vx - omega * ky) * (vx - omega * ky) + 
                                     (vy + omega * kx) * (vy + omega * kx));
        velocitys_[1].speed = -std::sqrt((vx - omega * ky) * (vx - omega * ky) + 
                                     (vy - omega * kx) * (vy - omega * kx));
        velocitys_[2].speed = std::sqrt((vx + omega * ky) * (vx + omega * ky) + 
                                     (vy - omega * kx) * (vy - omega * kx));
        velocitys_[3].speed = -std::sqrt((vx + omega * ky) * (vx + omega * ky) + 
                                     (vy + omega * kx) * (vy + omega * kx));
        
        velocitys_[0].dir = std::atan2(vy + omega * kx , vx - omega * ky) * RAD_2_DEG;
        velocitys_[1].dir = std::atan2(vy - omega * kx , vx - omega * ky) * RAD_2_DEG;
        velocitys_[2].dir = std::atan2(vy - omega * kx , vx + omega * ky) * RAD_2_DEG;
        velocitys_[3].dir = std::atan2(vy + omega * kx , vx + omega * ky) * RAD_2_DEG;

        if(std::fabs(vx) < 0.00001f &&
           std::fabs(vy) < 0.00001f &&
           std::fabs(omega) < 0.00001f){
            velocitys_[0].dir = 135.0f;
            velocitys_[1].dir = -135.0f;
            velocitys_[2].dir = -45.0f;
            velocitys_[3].dir = 45.0f;
        }

        for(uint8_t i = 0; i < kWheelNum; ++i){
            steerwheels_[i].setVelocity(velocitys_[i]);
        }


    }



private:


    static constexpr float kWheelbase = 1.0f;   //轴距
    static constexpr float kWheeltrack = 1.0f;  //轮距

    //ky,kx的值即便在同一个机器人上，也应该是可以不同的，其中，它的原点取决于你想让机器人自转时，绕机器人的哪个点旋转
    static constexpr float ky = kWheelbase / 2.0f;  //ky的含义是，点到x轴的距离
    static constexpr float kx = kWheeltrack / 2.0f; //kx的含义是，点到y轴的距离

    SteerWheel steerwheels_[kWheelNum]{};
    Velocity_t velocitys_[kWheelNum]{};

};