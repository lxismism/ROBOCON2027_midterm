/**
 * @file topic_pool.h
 * @author your name (you@domain.com)
 * @brief topic_pool里是发布订阅功能的消息包
 * @version 0.1
 * @date 2026-10-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#pragma pack(1)

typedef struct{
    float linear_x_;
    float linear_y_;
    float omega_;
} pub_chassis_cmd;

typedef struct{
  float roll_deg;   // 滚转角
  float pitch_deg; // 俯仰角
  float yaw_deg;  // 偏航角，绕Z轴旋转，范围 -π ~ +π
} pub_imu_data;

#pragma pack()