/**
 * @file djiMotor.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "math_utils.hpp"

#include "MotorBase.hpp"
#include "CanBus.hpp"

class C610Motor : public CanDevice , public MotorBase {
public:

    C610Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id, bool tx_is_extid, const PIDMode output_type,
              float reduction = 36.0f / 1.0f, float max_cmd = 10000.0f, float output_filter_rc = 0.0f,
              float speed_kp=0.0f, float speed_ki=0.0f, float speed_kd=0.0f, float speed_max_out = 10000.0f, float speed_Integral_Limit = 10000.0f,
              uint16_t pid_speed_improve = NONE,
              float deg_kp=0.0f, float deg_ki=0.0f, float deg_kd=0.0f, float deg_max_output=5000.0f, float deg_Integral_Limit=5000.0f,
              uint16_t pid_deg_improve = NONE)
              : CanDevice(manager, id, is_extid, tx_id, tx_is_extid){

                reduction_ratio_ = reduction;
                max_cmd_ = max_cmd;
                output_type_ = output_type;

                deg_speed_pid_.Kp = speed_kp;
                deg_speed_pid_.Ki = speed_ki;
                deg_speed_pid_.Kd = speed_kd;
                deg_speed_pid_.Maxout = speed_max_out;
                deg_speed_pid_.IntegralLimit = speed_Integral_Limit;

                deg_speed_pid_.Output_LPF_RC = output_filter_rc;
                deg_speed_pid_.Improve = pid_speed_improve;
                
                deg_pid_.Kp = deg_kp;
                deg_pid_.Ki = deg_ki;
                deg_pid_.Kd = deg_kd;
                deg_pid_.Maxout = deg_max_output;
                deg_pid_.IntegralLimit = deg_Integral_Limit;
                deg_pid_.Improve = pid_deg_improve;
                

            }

    void onRx(const uint8_t data[8], const uint8_t len) override {
        (void)len;

        /*data[0]~data[1]: 转子单圈角度，data[0]是高八位，data[1]是低八位*/
        uint16_t encoder = static_cast<uint16_t>((data[0] << 8U) | data[1]);
        if(!is_encoder_init) {
            encoder_offset_ = encoder;
            is_encoder_init = true;
        }

        if(encoder - last_encoder_ < -4096) round_cnt_++;
        else if(encoder - last_encoder_ > 4096) round_cnt_--;

        last_encoder_ = encoder;
        
        sum_deg_ = ((static_cast<float>(encoder - encoder_offset_) / 8192.0f +
             static_cast<float>(round_cnt_)) * 360.0f) / reduction_ratio_ ;

        single_deg_ =  math_utils::WrapAngle(sum_deg_, -180.0f, 180.0f);

        /*data[2]~data[3]: data[2]转子转速高8位, data[3]转子转速低8位*/
        int16_t raw_rpm = static_cast<int16_t>((data[2] << 8U) | data[3]);
        deg_speed_ = raw_rpm * RPM_2_DEG_PER_SEC / reduction_ratio_;

        /*data[4]~data[5]: data[4]实际输出转矩高8位，实际输出转矩低8位*/
        int16_t raw_current = static_cast<int16_t>((data[4] << 8U) | data[5]);
        torque_ = math_utils::Map(raw_current, -10000.0f, 10000.0f, -10.0f, 10.0f) * kCurToTorque;

    }

    float cmdTrans() { return cmd_; }


private:
    static constexpr float kCurToTorque = 0.18f;

    uint16_t encoder_offset_{};
    uint16_t last_encoder_{};
    int32_t round_cnt_;
    bool is_encoder_init = false;
    
};

class C620Motor : public CanDevice , public MotorBase {
public:
    C620Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id, bool tx_is_extid, const PIDMode output_type,
              float reduction = 3591.0f / 187.0f, float max_cmd = 20000.0f, float output_filter_rc = 0.0f,
              float speed_kp=0.0f, float speed_ki=0.0f, float speed_kd=0.0f, float speed_max_out = 20000.0f, float speed_max_IL = 20000.0f,
              uint16_t pid_speed_improve = NONE,
              float deg_kp=0.0f, float deg_ki=0.0f, float deg_kd=0.0f, float max_deg=5000.0f, float deg_max_IL=5000.0f,
              uint16_t pid_deg_improve = NONE)
              : CanDevice(manager, id, is_extid, tx_id, tx_is_extid){

                reduction_ratio_ = reduction;
                max_cmd_ = max_cmd;
                output_type_ = output_type;

                deg_speed_pid_.Kp = speed_kp;
                deg_speed_pid_.Ki = speed_ki;
                deg_speed_pid_.Kd = speed_kd;
                deg_speed_pid_.Maxout = speed_max_out;
                deg_speed_pid_.IntegralLimit = speed_max_IL;

                deg_speed_pid_.Output_LPF_RC = output_filter_rc;
                deg_speed_pid_.Improve = pid_speed_improve;
                
                deg_pid_.Kp = deg_kp;
                deg_pid_.Ki = deg_ki;
                deg_pid_.Kd = deg_kd;
                deg_pid_.Maxout = max_deg;
                deg_pid_.IntegralLimit = deg_max_IL;
                deg_pid_.Improve = pid_deg_improve;

            }


    void onRx(const uint8_t data[8], const uint8_t len) override{
        if(len < 8) return;     
        
        /*data[0]~data[1]: encoder data[0]为单圈内转子位置的高8位，data[1]为单圈内转子位置的低八位*/
        encoder_ = (uint16_t)((data[0] << 8) | data[1] );

        int16_t encoder_delta = encoder_ - last_encoder_;
        
        if(is_encoder_init){
        
            if(encoder_delta < -kCountPerHalfRound ) round_cnt_++;
            else if(encoder_delta > kCountPerHalfRound) round_cnt_--;
        
        } else {
            encoder_offset_ = encoder_;
            is_encoder_init = true;
        }
        last_encoder_ = encoder_;

        float raw_single_deg = (encoder_ * 360.0f / kCountPerRound) / reduction_ratio_;
        sum_deg_ = (static_cast<float>(round_cnt_) - 
                        static_cast<float>(encoder_offset_) / kCountPerRound) * 360.0f / reduction_ratio_
                         + raw_single_deg;
        single_deg_ = math_utils::WrapAngle(sum_deg_, -180.0f, 180.0f);
        

        /*data[2]~data[3]: data[2]转子RPM高8位，data[3]转子RPM低8位*/
        int16_t raw_rpm = static_cast<int16_t>((data[2] << 8) | data[3]);
        deg_speed_ = static_cast<float>(raw_rpm) * RPM_2_DEG_PER_SEC / reduction_ratio_;

        /*data[4]~data[5]: data[4]实际转矩电流高8位，data[5]实际按转矩电流低8位*/
        int16_t raw_tor_cur = static_cast<int16_t>((data[4] << 8) | data[5]);
        torque_ = static_cast<float>(raw_tor_cur) *20.0f * kCurToTorque / 16384.0f ;       //torque = current * k

        /*data[6]: 电机温度*/
        temperature_ = static_cast<float>(data[6]);

    }

    float cmdTrans() { return cmd_ * 16384.0f / 20000.0f;}


private:
    bool is_encoder_init = false;

    static constexpr float kCountPerRound = 8192.0f;
    static constexpr float kCountPerHalfRound = 4096.0f;
    static constexpr float kCurToTorque = 0.3f;  //转矩常数，单位N*m/A，这里是以DJI3508电机为准填的0.3，但是这玩意不同电机是不一样的，按理说应该作为一个初始化参数传进来，但是由于我们只用3508电机，所以就先这样吧。
    uint16_t encoder_offset_{0U};
    uint16_t encoder_{0U};
    uint16_t last_encoder_{0U};

    int32_t round_cnt_{0};


};

void packDJIMotorCanMsg(const uint32_t tx_id, const uint32_t *motor_ids,
                        const int16_t *commands,const uint8_t motor_count, 
                        uint8_t *data);