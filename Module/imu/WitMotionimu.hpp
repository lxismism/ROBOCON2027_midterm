/**
 * @file WitMotionimu.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include "topic_pool.h"

class WitMotionimu{
public:

enum class RxState : uint8_t {
    WAIT_HEADER = 0,
    WAIT_TYPE,
    WAIT_DATA,
    WAIT_SUM
};

uint8_t processByte(uint8_t byte){

    switch (state_) {
        case RxState::WAIT_HEADER : {
            if(byte == 0x55){
                rx_buf_[0] = byte;
                rx_idx_++;
                state_ = RxState::WAIT_TYPE;
            }
        } break;

        case RxState::WAIT_TYPE : {
            if(byte == 0x53){
                rx_buf_[1] = byte;
                rx_idx_++;
                state_ = RxState::WAIT_DATA;
            } else {
                resetState();
            }

        } break;

        case RxState::WAIT_DATA : {
            rx_buf_[rx_idx_] = byte;
            rx_idx_++;
            if(rx_idx_ >= (kRxBufLen - 1U)){
                state_ = RxState::WAIT_SUM;
            }
        } break;

        case RxState::WAIT_SUM : {
            uint8_t rx_sum = calculateSum(rx_buf_);
            resetState();
            if(rx_sum == byte){
                rx_buf_[10] = byte;
                
                int16_t raw_roll = static_cast<int16_t>(
                    (static_cast<uint16_t>(rx_buf_[3]) << 8) | 
                    static_cast<uint16_t>(rx_buf_[2]));

                int16_t raw_pitch = static_cast<int16_t>(
                    (static_cast<uint16_t>(rx_buf_[5]) << 8) | 
                    static_cast<uint16_t>(rx_buf_[4]));

                int16_t raw_yaw = static_cast<int16_t>(
                    (static_cast<uint16_t>(rx_buf_[7]) << 8) | 
                    static_cast<uint16_t>(rx_buf_[6]));

                //data[8]~data[9]是版本号,data[10]是校验和

                newest_data_.roll_deg = static_cast<float>(raw_roll) * 180.0f / 32768.0f;
                newest_data_.pitch_deg = static_cast<float>(raw_pitch) * 180.0f / 32768.0f;
                newest_data_.yaw_deg = static_cast<float>(raw_yaw) * 180.0f / 32768.0f;
              
                return 0x53;
            } 

        } break;

        default: {
            resetState();
        }
    }
    
    return 0U;

}

const pub_imu_data &getImuData() const { return newest_data_; }



private:

static constexpr uint8_t kRxBufLen = 11U;

RxState state_{RxState::WAIT_HEADER};
uint8_t rx_buf_[kRxBufLen]{};
uint8_t rx_idx_{};

void resetState(){
    state_ = RxState::WAIT_HEADER;
    rx_idx_ = 0;
}

uint8_t calculateSum(uint8_t rx_buf[kRxBufLen]){
    uint16_t sum = 0;
    for(uint8_t i = 0; i < kRxBufLen - 1U; ++i){
        sum += rx_buf[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

pub_imu_data newest_data_{};

};