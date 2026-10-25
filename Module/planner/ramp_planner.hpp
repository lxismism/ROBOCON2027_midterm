/**
 * @file speed_planner.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 我知道师兄在原框架为什么要用#ifdef和#endif
          ，用宏定义作为特色功能的开关了，如果框架的使用者不想使用这个功能，
            直接不定义就好了，到时候可以少传几个参，这个功能启用了就要在所有包含这个类型的对象那里传参,真卡。
 * @version 0.1
 * @date 2026-10-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "bsp_dwt.h"
#include "math_utils.hpp"

#define MAX_DT 0.020f


//速度规划这个名字具有极强的误导性，故在此改成S型斜坡规划
//梯形规划的本质：生成且只生成一阶导的斜坡
//S型规划的本质:不但能生成一阶导的斜坡，并且生成二阶导的斜坡（积分后以一阶导的首尾出现圆弧表现出来），但不生成之后的导数的斜坡。
class S_RampPlanner{
public:
    S_RampPlanner() = default;    

    void init(float jerk){
        jerk_max_ = jerk;
    }

float update(float v_tar, float a_tar) {
    const float dt = DWT_GetDeltaT(&DWT_CNT_);
    if(dt > MAX_DT || dt < 1e-6f){
        return v_ref_;
    }

    float a_des = 0.0f;
    float v_diff = v_tar - v_ref_; // 速度差

    // === 核心修改：基于刹车距离的前瞻逻辑 ===
    // 计算当前加速度降为0所需的速度余量（速度空间中的刹车距离）
    float brake_dist = (a_ref_ * a_ref_) / (2.0f * jerk_max_);

    if (v_diff > 0) {
        // 目标在前方（加速区）
        if (v_diff <= brake_dist) {
            a_des = 0.0f; // 进入刹车区，停止加速，让加速度自然降为0
        } else {
            a_des = a_tar; // 还没到刹车区，全力加速
        }
    } else if (v_diff < 0) {
        // 目标在后方（减速区）
        if (-v_diff <= brake_dist) {
            a_des = 0.0f; // 进入反向刹车区
        } else {
            a_des = -a_tar; // 全力减速
        }
    } else {
        a_des = 0.0f; // 刚好到达
    }

    // 限制加速度的变化率 (Jerk)
    float da = a_des - a_ref_;
    da = math_utils::Clamp(da, -jerk_max_ * dt, jerk_max_ * dt);
    a_ref_ += da;

    // 积分出速度
    v_ref_ += a_ref_ * dt;
    
    return v_ref_;
}

    void reset(){
        v_ref_ = 0.0f;
        a_ref_ = 0.0f;
    }

private:
    float jerk_max_{20000.0f};      //这个应该是加加速度的物理极限,电流环的变化的最大斜率         
    
    //计算公式a_max = i_max * kItoTorque / m。          tm这玩意就是傻逼，这行计算公式的前提是a的含义是电机加速度
    //当规划对象是位置，即a的含义为速度时，这就不是这样了，其实这个a这个时候就是达妙电机位置速度模式的给定速度。
    
    float a_ref_{};         
    float v_ref_{};         //根据v_ref = ∫adt生成0到t积分段每一时刻的速度值
    uint32_t DWT_CNT_{};

};