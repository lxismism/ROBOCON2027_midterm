/**
 * @file pump_valve.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "gpio.h"


class BoolDevice{
public:
    BoolDevice(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin) : 
        GPIO_Port_(GPIO_Port), GPIO_Pin_(GPIO_Pin){};

    void On(void){
        HAL_GPIO_WritePin(GPIO_Port_, GPIO_Pin_, GPIO_PIN_SET);
    }
    void Off(void){
        HAL_GPIO_WritePin(GPIO_Port_, GPIO_Pin_, GPIO_PIN_RESET);
    }

private:
    GPIO_TypeDef* GPIO_Port_;
    uint16_t GPIO_Pin_;
};