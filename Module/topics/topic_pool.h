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
    float omega_;   //注意了！！！单位是弧度！rad
} pub_chassis_cmd;

typedef struct{
  float roll_deg;   // 滚转角
  float pitch_deg; // 俯仰角
  float yaw_deg;  // 偏航角，绕Z轴旋转，范围 -180~180
} pub_imu_data;

typedef struct {
  float lift_deg;
  float tripod_deg;   //单位：°
  float extend_deg;
  bool flip;
  bool suction_on;   //0为停，1为吸
} pub_pickhand_cmd;

typedef struct{
  float lift_deg;
  float pick;
} pub_weaponhand_cmd;

#pragma pack()