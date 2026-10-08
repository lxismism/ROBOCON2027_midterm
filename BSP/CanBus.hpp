/**
 * @file CanBus.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-28
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once
#include "stm32h7xx_hal.h"
#include <cstddef>
#include <cstdint>
#include "lockfree_queue.hpp"
#include <atomic>

#define FDCAN_BUS_CNT 3U

void canFilterInit(FDCAN_HandleTypeDef *hcan, uint32_t idType, 
                       uint32_t id, uint32_t maskId, uint32_t fifo, uint32_t filterIndex);

void bspCanInit(FDCAN_HandleTypeDef *hcan);

class CanBus;

class CanDevice{
public:
    CanDevice(CanBus *manager, uint32_t id = 0, bool is_extid = false, uint32_t tx_id = 0,
              bool tx_is_extid = false)
              : manager_(manager), id_(id), is_extid_(is_extid), tx_id_(tx_id),
                tx_is_extid_(tx_is_extid){}
    
    virtual ~CanDevice() = default;
    
    virtual void onRx(const uint8_t data[8], const uint8_t len){
        (void)data;
        (void)len;
    }

    virtual bool buildTx(uint8_t data[8]){
        (void)data;
        return false;
    }

    virtual bool matchRx(const FDCAN_RxHeaderTypeDef &rx_header) const {
        const bool frame_is_ext = (rx_header.IdType == FDCAN_EXTENDED_ID);
        if(frame_is_ext != is_extid_) return false;
        return (rx_header.Identifier == id_);
    }

    uint32_t getID(){ return id_; }

    

protected:
    CanBus *manager_{nullptr};
    uint32_t id_;
    bool is_extid_;
    uint32_t tx_id_;
    bool tx_is_extid_;

};

class CanBus{
public:
    enum class Type : uint8_t { STANDARD = 0, EXTENDED = 1};
    enum class CanError : uint8_t { OK = 0, ERROR = 1, DEVICEFULL = 2};

    typedef struct __attribute__((packed)){
        uint32_t id;
        Type type;
        uint32_t dlc;
        uint8_t data[8];
    }ClassicPack;

    CanBus(FDCAN_HandleTypeDef &impl) : impl_(impl){}

    CanError init(void);
    CanError registerDevice(CanDevice *device);
    CanError addCanMsg(const ClassicPack &pack);

    void processRxInterrupt(uint32_t fifo);
    void processTxInterrupt();

    uint32_t hardwareTxQueueEmptySize(){
        return HAL_FDCAN_GetTxFifoFreeLevel(&impl_);
    }

    void txService();
    static CanBus *instanceByIndex(uint8_t idx);

private:
    FDCAN_HandleTypeDef &impl_;
    Algorithm::MpscQueue<ClassicPack, 8> tx_queue_;
    std::atomic<uint32_t> tx_lock_{0};
    std::atomic<uint32_t> tx_pend_{0};

    static CanBus *map_[FDCAN_BUS_CNT];
    static constexpr uint8_t kMaxDevice = 6;
    CanDevice *device_[kMaxDevice];

};