/**
 * @file math_utils.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once
#include <cmath>

namespace math_utils {

inline float Clamp(float val, float min, float max){
    return (val >= max)? max : ((val <= min)? min : val);
}

inline float WrapAngle(float val, float min, float max){
    float range_size = max - min;
    if(range_size <= 0.0f) return min;
    val = std::fmodf(val - min , range_size);
    if(val < 0.0f) val += range_size;
    return val + min; 
}

inline float Map(float val, float in0, float in1, float out0, float out1){

    float range_in = in1 - in0;
    float range_out = out1 - out0;

    if(range_in == 0.0f) return out0;

    float offset_in = val - in0;
    return out0 + offset_in * range_out / range_in;

}

}   //namespace math_utils