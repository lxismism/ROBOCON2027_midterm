/**
 * @file Motor.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "Motor.hpp"

static uint8_t djiSlotFromidx(const uint32_t tx_id, const uint32_t rx_id){
    if(tx_id == 0x200){
        if(rx_id >= 0x201 && rx_id <= 0x204){
            return rx_id - 0x201;
        }
        return -1;
    }
    else if(tx_id == 0x1FF){
        if(rx_id >= 0x205 && rx_id <= 0x208){
            return rx_id - 0x205;
        }
        return -1;
    }
    return -1;
}

void packDJIMotorCanMsg(const uint32_t tx_id, const uint32_t motor_ids[],
                        const int16_t commands[], const uint8_t motor_count,
                        uint8_t data[8], uint8_t &len){
    len = 8;
    if(motor_count <= 0) return;

    for(uint8_t i = 0; i < 8; ++i){
        data[i] = 0;
    }

    uint8_t occupied_slots = 0;     //位图

    for(uint8_t i = 0; i < motor_count; ++i){
        
        if(motor_ids[i] == 0) continue;

        const int8_t slot = djiSlotFromidx(tx_id, motor_ids[i]);
        
        if(slot < 0 || slot > 3) continue;

        const uint8_t slot_mask = 1U << static_cast<uint8_t>(slot);
        
        if((occupied_slots & slot_mask) != 0) continue;   //同一个slot再次出现，直接pass，期望中一个槽是只能出现一次的

        const int16_t cmd = commands[i];
        data[slot * 2] = static_cast<uint8_t>(cmd >> 8);
        data[slot * 2 + 1] = static_cast<uint8_t>(cmd & 0xFF);
        occupied_slots |= slot_mask;

    }
}