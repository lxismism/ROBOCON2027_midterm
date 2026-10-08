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

    PID_t* getDegSpeedPID(void) { return &deg_speed_pid_;}
    PID_t* getDegPID(void) { return &deg_pid_;}

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

    float getRefDegSpeed() { return ref_deg_speed_; }
    float getRefDeg() { return ref_deg_; }

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

    float single_deg_{0.0f};
    float sum_deg_{0.0f};
    float deg_speed_{0.0f};
    float torque_{0.0f};
    float temperature_{0.0f};

    PID_t deg_speed_pid_{};
    PID_t deg_pid_{};
    PIDMode output_type_{};
};

class C620Motor : public CanDevice , public MotorBase {
public:
    C620Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id, bool tx_is_extid, const PIDMode output_type,
              float reduction = 3591.0f / 187.0f, float max_cmd = 20000.0f, float output_filter_rc = 0.0f,
              float speed_kp=0.0f, float speed_ki=0.0f, float speed_kd=0.0f, float speed_max_out = 20000.0f, float speed_max_IL = 20000.0f,
              float pid_speed_improve = NONE,
              float deg_kp=0.0f, float deg_ki=0.0f, float deg_kd=0.0f, float max_deg=2000.0f, float deg_max_IL=2000.0f,
              float pid_deg_improve = NONE)
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

        single_deg_ = (encoder_ * 360.0f / kCountPerRound) / reduction_ratio_;
        sum_deg_ = (static_cast<float>(round_cnt_) - 
                        static_cast<float>(encoder_offset_) / kCountPerRound) * 360.0f / reduction_ratio_
                         + single_deg_;

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

    bool buildTx(uint8_t data[8]) override {
        
        return false;
    }

    

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
                        uint8_t *data, uint8_t &len);
                        

class VESCMotor : public CanDevice , public MotorBase {
public:
    enum Command_ID {
        CAN_PACKET_SET_DUTY                     = 0U,
        CAN_PACKET_SET_CURRENT                  = 1U,
        CAN_PACKET_SET_CURRENT_BRAKE            = 2U,
        CAN_PACKET_SET_RPM                      = 3U,
        CAN_PACKET_SET_POS                      = 4U,
        CAN_PACKET_SET_CURRENT_REL              = 10U,
        CAN_PACKET_SET_CURRENT_BRAKE_REL        = 11U,
        CAN_PACKET_SET_CURRENT_HANDBRAKE        = 12U,
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL    = 13U,

    };

    enum Status_ID {
        CAN_PACKET_STATUS   = 9U,
        CAN_PACKET_STATUS_2 = 14U,
        CAN_PACKET_STATUS_3 = 15U,
        CAN_PACKET_STATUS_4 = 16U,
        CAN_PACKET_STATUS_5 = 27U,
        CAN_PACKET_STATUS_6 = 58U
    };

    VESCMotor(CanBus *manager, uint32_t id, const PIDMode output_type, 
        const uint8_t command_id = CAN_PACKET_SET_CURRENT, const uint8_t status_id = CAN_PACKET_STATUS,
        float num_of_pole_pairs = 21.0f, float reduction = 1.0f, float max_cmd = 20000.0f, float output_filter_rc = 0.0f,
        float speed_kp=0.0f, float speed_ki=0.0f, float speed_kd=0.0f, float speed_max_out = 20000.0f, float speed_max_IL = 20000.0f,
        float pid_speed_improve = NONE,
        float deg_kp=0.0f, float deg_ki=0.0f, float deg_kd=0.0f, float max_deg=2000.0f, float deg_max_IL=2000.0f,
        float pid_deg_improve = NONE)
        : CanDevice(manager, id, true, id, true){

                num_of_pole_pairs_ = num_of_pole_pairs;
                reduction_ratio_ = reduction;
                max_cmd_ = max_cmd;
                output_type_ = output_type;
                command_id_ = command_id;
                status_id_ = status_id;

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


    uint8_t getCommandID(){ return command_id_; }

    bool buildTx(uint8_t data[8]) override{

        int32_t command = static_cast<int32_t>(cmdTrans());
        data[0] = static_cast<uint8_t>((command >> 24) & 0xFF);
        data[1] = static_cast<uint8_t>((command >> 16) & 0xFF);
        data[2] = static_cast<uint8_t>((command >>  8) & 0xFF);
        data[3] = static_cast<uint8_t>(command & 0xFF);

        return true;

    }

    void onRx(const uint8_t data[8], const uint8_t len) override {
        (void)len;
        /*data[0]~data[3]: ERPM 电转速*/
        int32_t ERPM = static_cast<int32_t>((data[0] << 24) |
                                            (data[1] << 16) |
                                            (data[2] <<  8) |
                                            data[3]);
        /*RPM = ERPM / num_of_pole_pairs(极对数)*/
        deg_speed_ = static_cast<float>(ERPM)  * RPM_2_DEG_PER_SEC / num_of_pole_pairs_;
        
        /*data[4]~data[5]: 电流，缩放因子为10*/
        int16_t raw_current = static_cast<int16_t>((data[4] << 8) | data[5]);
        current_ = raw_current / 10.0f;

        /*data[6]~data[7]: 占空比*/
        int16_t raw_duty = static_cast<int16_t>(data[6] << 8 | data[7]);
        duty_cycle_ = raw_duty / 1000.0f;
    }

    bool matchRx(const FDCAN_RxHeaderTypeDef &rx_header) const override{
        const bool frame_is_ext = (rx_header.IdType == FDCAN_EXTENDED_ID);
        if(frame_is_ext != is_extid_) return false;
        const uint8_t status_ID = (rx_header.Identifier >> 8) & 0xFFU;
        const uint8_t id = (rx_header.Identifier & 0xFFU);
        
        return ((status_ID == status_id_) && (id == id_));
    }

    float cmdTrans() { return cmd_ ; }
    

private:
    uint8_t command_id_;
    uint8_t status_id_;

    float num_of_pole_pairs_;
    float current_;
    float duty_cycle_;


};