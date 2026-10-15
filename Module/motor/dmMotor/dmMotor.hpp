/**
 * @file dmMotor.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "CanBus.hpp"
#include "MotorBase.hpp"
#include "math_utils.hpp"
class DM4310Motor : public CanDevice, public MotorBase {
public:
    enum DM4310Mode : uint8_t {
        MIT =          0x01,
        PosWithSpeed = 0x02,
        Speed =        0x03,
        Psi =          0x04
    };

    //这里起master_id和can_id主要是为了和DMtool上面对齐
    DM4310Motor(CanBus *manager, uint32_t master_id, uint32_t can_id, 
              DM4310Mode control_mode = PosWithSpeed, float reduction = 10.0f / 1.0f, float max_cmd = 10000.0f)
              : CanDevice(manager, master_id, false, can_id, false){

                reduction_ratio_ = reduction;
                max_cmd_ = max_cmd;
                output_type_ = OutputType::NONE;
                control_mode_ = control_mode;

                switch (control_mode) {
                    case PosWithSpeed : tx_id_ = kOffsetPoswithSpeed + can_id; break;
                    default: tx_id_ = can_id;
                }

            }


    void onRx(const uint8_t data[8], uint8_t len) override {
        if(len < 8) return;

        /*data[0] = ID | (err >> 4)，意为data[0]是由两个4bit的数据，ID和err组成，其中err在高4位*/
        ID_ = data[0] & 0x0F;
        error_state_ = data[0] >> 4U;

        is_enabled_ = (error_state_ == kStatusEnabled);

        /*data[1]~data[2]: pos的高8位和低8位*/
        const uint16_t raw_pos = static_cast<uint16_t>((data[1] << 8U) | data[2]);
        const float raw_rad = math_utils::Map(static_cast<float>(raw_pos), 0.0f, 65535.0f, kRadMin, kRadMax);
        sum_deg_ = raw_rad * RAD_2_DEG;
        single_deg_ = math_utils::WrapAngle(sum_deg_, -180.0f, 180.0f);

        /*data[3]+data[4]高4位，组成一个12位的数据表示角速度*/
        const uint16_t raw_speed = static_cast<uint16_t>((data[3] << 4U) | (data[4] >> 4U));
        const float raw_radspeed = math_utils::Map(static_cast<float>(raw_speed), 
            0.0f, 4095.0f, kRadspeedMin, kRadspeedMax);
        deg_speed_ = raw_radspeed * RAD_2_DEG;

        /*data[4]低4位 + data[5]，组成一个12位的数据表示扭矩*/
        const uint16_t raw_torque = static_cast<uint16_t>(((data[4] & 0x0F) << 8U) | data[5]);
        torque_ = math_utils::Map(static_cast<float>(raw_torque), 
            0.0f, 4095.0f, kTorMin, kTorMax);

        /*data[6]: MOS温度; D[7]: 线圈温度*/
        t_mos_ = data[6];
        t_rotor_ = data[7];
        temperature_ = static_cast<float>(t_rotor_);

    }

    bool buildTx(uint8_t data[8]) override {
        if(isOffline() || !is_enabled_){
            uint32_t now = HAL_GetTick();
            if((now - last_enable_ms_) < kEnableIntervalMs){
                return false;     //本次不发帧，发使能帧的频率不要太快
            }
            last_enable_ms_ = now;
            packEnableCmd(data);
            return true;
            
        }
        switch (control_mode_) {
            case PosWithSpeed : {
                packPosWithSpeedCmd(data);
                return true;
            }
            default:        return false;
        }
    }



private:
    static constexpr float kRadMin = -12.5f;   // rad
    static constexpr float kRadMax =  12.5f;
    static constexpr float kRadspeedMin = -30.0f;   // rad/s
    static constexpr float kRadspeedMax =  30.0f;
    static constexpr float kTorMin = -10.0f;   // N·m
    static constexpr float kTorMax =  10.0f;

    static constexpr uint8_t kStatusEnabled = 1U;
    uint32_t last_enable_ms_{};
    uint8_t enable_clear_toggle_{};

    static constexpr uint32_t kOffsetPoswithSpeed = 0x100;

    static constexpr uint8_t kCmdEnable = 0xFCU;
    static constexpr uint8_t kCmdClearError = 0xFBU;

    static constexpr uint32_t kEnableIntervalMs = 20U;

    DM4310Mode control_mode_{PosWithSpeed};
    uint8_t error_state_{0U};
    uint8_t ID_{0U};
    uint8_t t_mos_{0};
    uint8_t t_rotor_{0};
    bool is_enabled_{false};
    

    void packEnableCmd(uint8_t data[8]){

        const uint8_t cmd = (enable_clear_toggle_ == 0U) ?
            kCmdClearError : kCmdEnable;
        enable_clear_toggle_ ^= 1U;

        for(uint8_t i = 0; i < 7; ++i){
            data[i] = 0xFF;
        }
        data[7] = cmd;
    }

    void packPosWithSpeedCmd(uint8_t data[8]){
        float ref_rad = ref_deg_ * DEG_2_RAD;
        float ref_rad_speed = ref_deg_speed_ * DEG_2_RAD;

        const uint8_t *p_des = reinterpret_cast<uint8_t*>(&ref_rad);
        const uint8_t *v_des = reinterpret_cast<uint8_t*>(&ref_rad_speed);

        for(uint8_t i = 0; i < 4; ++i){
            data[i] = p_des[i];
            data[i+4] = v_des[i];
        }

    }

};