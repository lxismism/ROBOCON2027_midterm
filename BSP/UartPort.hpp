/**
 * @file UartPort.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */   


#pragma once

#include "stm32h7xx_hal.h"
#include <cstddef>
#include <cstdint>
#include "double_buffer.hpp"
#include "lockfree_queue.hpp"

class UartPort{
public:
    static constexpr size_t kPacketPayloadSize = 64;
    static constexpr size_t kRxQueueDepth = 8;

    struct Packet{
        uint16_t len{0};
        uint8_t data[kPacketPayloadSize]{};
    };

    using RxCallback = void(*)(const uint8_t *data, size_t len, void *user);
    UartPort(UART_HandleTypeDef *huart, uint8_t *rx_dma_buf,
             size_t rx_dma_buf_size, uint8_t *tx_dma_buf = nullptr,
             size_t tx_dma_buf_size = 0, RxCallback cb = nullptr,
             void *cb_user = nullptr);

    HAL_StatusTypeDef startRxDmaIdle();
    bool Read(Packet &packet);
    void onRxEvent();
    void onError(uint32_t error_code);
    HAL_StatusTypeDef write(const uint8_t *data, size_t len, uint32_t timeout_ms = 10);
    HAL_StatusTypeDef writeDma(const uint8_t *data, size_t len);
    UART_HandleTypeDef *handle() const { return huart_; }
    static UartPort* fromHandle(UART_HandleTypeDef *huart);
    void onTxCplt();


    bool txBusy() const { return tx_busy_; }



private:
    UART_HandleTypeDef *huart_{nullptr};
    volatile bool tx_busy_{false};
    uint8_t *rx_dma_buf_{nullptr};
    size_t rx_dma_buf_size_{0};
    RxCallback rx_callback_{nullptr};
    void *rx_callback_user_{nullptr};
    size_t last_rx_pos_{0};
    
    Algorithm::RawData tx_dma_raw_;
    Algorithm::DoubleBuffer tx_dma_buf_;
    uint8_t tx_fallback_[2] = {0,0};
    bool tx_use_double_buffer_{false};

    using RxPacketQueue = Algorithm::MpscQueue<Packet, kRxQueueDepth>;
    RxPacketQueue rx_queue_;

    static constexpr size_t kMaxMap = 10;
    static UartPort *map_[kMaxMap];

};