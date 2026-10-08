/**
 * @file CanBus.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-28
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "Canbus.hpp"
#include "main.h"
#include "fdcan.h"
#include <cstdint>

namespace {

uint8_t dlcToByte(uint32_t dlc){
    switch (dlc) {
        case FDCAN_DLC_BYTES_0 : return 0;        
        case FDCAN_DLC_BYTES_1 : return 1;
        case FDCAN_DLC_BYTES_2 : return 2;
        case FDCAN_DLC_BYTES_3 : return 3;
        case FDCAN_DLC_BYTES_4 : return 4;
        case FDCAN_DLC_BYTES_5 : return 5;
        case FDCAN_DLC_BYTES_6 : return 6;
        case FDCAN_DLC_BYTES_7 : return 7;        
        case FDCAN_DLC_BYTES_8 : 
        default : return 8;
    }
}

int8_t busIndexFromHandle(FDCAN_HandleTypeDef *hfdcan){
    if(hfdcan == &hfdcan1) return 0;
    if(hfdcan == &hfdcan2) return 1;
    if(hfdcan == &hfdcan3) return 2;
    return -1;
}

}

CanBus* CanBus::map_[FDCAN_BUS_CNT] = {nullptr, nullptr, nullptr};

void canFilterInit(FDCAN_HandleTypeDef *hcan, uint32_t idType,
                   uint32_t id, uint32_t maskId, uint32_t fifo, uint32_t filterIndex){
    FDCAN_FilterTypeDef sFilterConfig = {0};
    
    sFilterConfig.FilterConfig = fifo;
    sFilterConfig.FilterID1 = id;
    sFilterConfig.FilterID2 = maskId;
    sFilterConfig.FilterIndex = filterIndex;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.IdType = idType;
    sFilterConfig.IsCalibrationMsg = 0;
    sFilterConfig.RxBufferIndex = 0; 

    if(HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        Error_Handler();
    }
    
    if(HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_REJECT, FDCAN_REJECT, 
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)
                                 != HAL_OK){
        Error_Handler();
    }

}

void bspCanInit(FDCAN_HandleTypeDef *hcan){
    if(HAL_FDCAN_ActivateNotification(hcan, 
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                      FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                                      FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE |
                                      FDCAN_IT_TX_FIFO_EMPTY,
                                      0) != HAL_OK){
        Error_Handler();
    }


    if(HAL_FDCAN_Start(hcan) != HAL_OK){
        Error_Handler();
    }

}

CanBus *CanBus::instanceByIndex(uint8_t idx){
    if(idx >= FDCAN_BUS_CNT) return nullptr;
    return map_[idx];
}

CanBus::CanError CanBus::init(void){
    const int8_t idx = busIndexFromHandle(&impl_);
    if(idx < 0){
        return CanError::ERROR;
    }
    map_[idx] = this;
    for(uint8_t i = 0; i < kMaxDevice; ++i){
        device_[i] = nullptr;
    }
    
    return CanError::OK;
}

CanBus::CanError CanBus::registerDevice(CanDevice *device){
    if(device == nullptr) return CanError::ERROR;

    for(uint8_t i = 0; i < kMaxDevice; ++i){
        if(device_[i] == device){
            return CanError::OK;
        }
    }

    for(uint8_t i = 0; i < kMaxDevice; ++i){
        if(device_[i] == nullptr){
            device_[i] = device;
            return CanError::OK;
        }
    }

    return CanError::DEVICEFULL;

}

CanBus::CanError CanBus::addCanMsg(const ClassicPack &pack){
    CanError result = CanError::OK;
    
    if(tx_queue_.TryPush(pack) != Algorithm::QueueError::OK){
        return result = CanError::ERROR;
    }

    txService();
    return result;
}

void CanBus::processRxInterrupt(uint32_t fifo){
    FDCAN_RxHeaderTypeDef rx_header = {0};
    uint8_t data[8] = {0};

    if(HAL_FDCAN_GetRxMessage(&impl_, fifo, &rx_header, data) != HAL_OK){
        return;
    }

    const uint8_t len = dlcToByte(rx_header.DataLength);
    for(uint8_t i = 0; i < kMaxDevice; ++i){
        if(device_[i] == nullptr) continue;
        
        if(device_[i]->matchRx(rx_header)){
            device_[i]->onRx(data, len);
        }
    }

}

void CanBus::txService(){
    if(tx_lock_.exchange(1, std::memory_order_acq_rel) != 0){
        tx_pend_.store(1, std::memory_order_release);
        return;
    }
    do{
        tx_pend_.store(0, std::memory_order_release);

        while (HAL_FDCAN_GetTxFifoFreeLevel(&impl_) > 0U) {
            CanBus::ClassicPack pakcet = {0};
            if(tx_queue_.TryPop(pakcet) != Algorithm::QueueError::OK){
                break;
            }

            FDCAN_TxHeaderTypeDef tx_header = {0};
            tx_header.Identifier = pakcet.id;
            tx_header.IdType = (pakcet.type == Type::EXTENDED) ? FDCAN_EXTENDED_ID
                                                                 : FDCAN_STANDARD_ID;
            tx_header.TxFrameType = FDCAN_DATA_FRAME;
            tx_header.DataLength = pakcet.dlc;
            tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
            tx_header.BitRateSwitch = FDCAN_BRS_OFF;
            tx_header.FDFormat = FDCAN_CLASSIC_CAN;
            tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            tx_header.MessageMarker = 0;

            if(HAL_FDCAN_AddMessageToTxFifoQ(&impl_, &tx_header, pakcet.data) != HAL_OK){
                tx_pend_.store(1, std::memory_order_release);
                break;
            }
        }
    }while(tx_pend_.exchange(0, std::memory_order_acq_rel) != 0);

    tx_lock_.store(0, std::memory_order_release);
}

void CanBus::processTxInterrupt(){ txService(); }

/*----------------Callbacks------------------*/
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0){
        return;
    }
    const int8_t idx = busIndexFromHandle(hfdcan);
    CanBus *bus = (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
    if(bus != nullptr){
        bus->processRxInterrupt(FDCAN_RX_FIFO0);
    }

}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0){
        return;
    }
    const int8_t idx = busIndexFromHandle(hfdcan);
    CanBus *bus = (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
    if(bus != nullptr){
        bus->processRxInterrupt(FDCAN_RX_FIFO1);
    }
}

extern "C" void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan){
    const int8_t idx = busIndexFromHandle(hfdcan);
    CanBus *bus = (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
    if(bus != nullptr){
        bus->txService();
    }
}

extern "C" void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan){
    (void)hfdcan;
    while(1){}
}


