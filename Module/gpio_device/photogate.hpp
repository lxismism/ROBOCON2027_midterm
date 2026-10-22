/**
 * @file photogate.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once
#include "gpio.h"

class PhotoGate{
public:
    PhotoGate(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
        : gpio_port_(GPIO_Port), gpio_pin_(GPIO_Pin){}

    bool isBlocked(){ return state_; }

    bool hasRisingEdge(){ return ((!last_state_) && state_); }
    bool hasFallingEdge(){ return (last_state_ && (!state_)); }
    
    void update(){
        if(is_first_frame_){

            state_ = (HAL_GPIO_ReadPin(gpio_port_, gpio_pin_) == GPIO_PIN_RESET);
            last_state_ = state_;
            is_first_frame_ = false;
        } else {
            last_state_ = state_;
            state_ = (HAL_GPIO_ReadPin(gpio_port_, gpio_pin_) == GPIO_PIN_RESET);
        }
    }


private:
    bool state_{false};     //无遮挡是0，有遮挡是1
    bool last_state_{false};
    bool is_first_frame_{true};
    GPIO_TypeDef* gpio_port_;
    uint16_t gpio_pin_;

};