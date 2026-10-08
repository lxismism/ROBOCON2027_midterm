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

#define PI 3.1415926535f

class SteerChassis{
public:
    SteerChassis(const std::array<MotorBase* , 4> &dirmotors,
                 const std::array<MotorBase* , 4> &drivemotors) :
    dirmotors_(dirmotors), drivemotors_(drivemotors){}

    void run(const pub_chassis_cmd &cmd){
        const float vx = cmd.linear_x_;
        const float vy = cmd.linear_y_;
        const float omega = cmd.omega_;

        const float v_ru = std::sqrt((vx - omega * ky) * (vx - omega * ky) + 
                                     (vy + omega * kx) * (vy + omega * kx));
        const float v_lu = -std::sqrt((vx - omega * ky) * (vx - omega * ky) + 
                                     (vy - omega * kx) * (vy - omega * kx));
        const float v_ld = std::sqrt((vx + omega * ky) * (vx + omega * ky) + 
                                     (vy - omega * kx) * (vy - omega * kx));
        const float v_rd = -std::sqrt((vx + omega * ky) * (vx + omega * ky) + 
                                     (vy + omega * kx) * (vy + omega * kx));
        
        const float rad_ru = std::atan2(vy + omega * kx , vx - omega * ky);
        const float rad_lu = std::atan2(vy - omega * kx , vx - omega * ky);
        const float rad_ld = std::atan2(vy - omega * kx , vx + omega * ky);
        const float rad_rd = std::atan2(vy + omega * kx , vx + omega * ky);


        //最小转动角度
        deg_output_[0] = rad_ru * RAD_2_DEG;
        deg_output_[1] = rad_lu * RAD_2_DEG;
        deg_output_[2] = rad_ld * RAD_2_DEG;
        deg_output_[3] = rad_rd * RAD_2_DEG;

        degspeed_output_[0] = v_ru / kWheelRadmeter * RAD_2_DEG;
        degspeed_output_[1] = v_lu / kWheelRadmeter * RAD_2_DEG;
        degspeed_output_[2] = v_ld / kWheelRadmeter * RAD_2_DEG;
        degspeed_output_[3] = v_rd / kWheelRadmeter * RAD_2_DEG;


//TODO:最小路径


        /*输出degree为基本单位的角度和角速度*/
        for(uint8_t i = 0; i < kWheelNum; ++i){
            dirmotors_[i]->setMotorDeg(deg_output_[i]);
            drivemotors_[i]->setMotorDegSpeed(degspeed_output_[i]);
        }


    }



private:

    static constexpr float kWheelDiameter = 0.104f;
    static constexpr float kWheelRadmeter = kWheelDiameter / 2.0f;
    static constexpr float kWheelCirmeter = PI * kWheelDiameter;
    static constexpr uint8_t kWheelNum = 4U;

    static constexpr float kWheelbase = 1.0f;   //轴距
    static constexpr float kWheeltrack = 1.0f;  //轮距

    //ky,kx的值即便在同一个机器人上，也应该是可以不同的，其中，它的原点取决于你想让机器人自转时，绕机器人的哪个点旋转
    //这里ky，kx除以2纯属偶然，就是碰巧，毕竟师兄的车是正方向底盘
    static constexpr float ky = kWheelbase / 2.0f;  //ky的含义是，点到x轴的距离
    static constexpr float kx = kWheeltrack / 2.0f; //kx的含义是，点到y轴的距离

    std::array<MotorBase*, kWheelNum> dirmotors_{};
    std::array<MotorBase*, kWheelNum> drivemotors_{};

    std::array<float, kWheelNum> deg_output_{};
    std::array<float, kWheelNum> degspeed_output_{};
    std::array<float, kWheelNum> deg_current{};

//TODO:更新当前角度的函数

};