/**
 * @file vescMotor.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "MotorBase.hpp"
#include "CanBus.hpp"

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