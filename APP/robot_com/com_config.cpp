/**
 * @file com_config.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "com_config.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "usart.h"
#include "fdcan.h"

#include "topics.hpp"
#include "memory_map.h"
#include "UartPort.hpp"
#include "Canbus.hpp"
#include "Motor.hpp"
#include "topic_pool.h"
#include <array>
#include "chassis_solution.hpp"

/*-------------------------------------fdcan1----------------------------------------*/

osThreadId_t can1Send_TaskHandle;
CanBus fdcan1_bus(hfdcan1);


/*-------------------------------------fdcan1----------------------------------------*/



/*-------------------------------------fdcan2----------------------------------------*/

osThreadId_t can2Send_TaskHandle;
CanBus fdcan2_bus(hfdcan2);

VESCMotor chassis_drivemotor1(&fdcan2_bus, 101, MotorBase::PIDMode::SPEED, 
    VESCMotor::CAN_PACKET_SET_CURRENT, VESCMotor::CAN_PACKET_STATUS,
    21.0f, 1.0f, 20000.0f, 0.005f,
    1.5f, 0.5f, 0.0f, 20000.0f, 20000.0f,
    OutputFilter);

VESCMotor chassis_drivemotor2(&fdcan2_bus, 102, MotorBase::PIDMode::SPEED, 
    VESCMotor::CAN_PACKET_SET_CURRENT,VESCMotor::CAN_PACKET_STATUS,
    21.0f,1.0f, 20000.0f, 0.005f,
    1.5f, 0.5f, 0.0f, 20000.0f, 20000.0f,
    OutputFilter);

VESCMotor chassis_drivemotor3(&fdcan2_bus, 103, MotorBase::PIDMode::SPEED, 
    VESCMotor::CAN_PACKET_SET_CURRENT, VESCMotor::CAN_PACKET_STATUS,
    21.0f,1.0f, 20000.0f, 0.005f,
    1.5f, 0.5f, 0.0f, 20000.0f, 20000.0f,
    OutputFilter);

VESCMotor chassis_drivemotor4(&fdcan2_bus, 104, MotorBase::PIDMode::SPEED, 
    VESCMotor::CAN_PACKET_SET_CURRENT, VESCMotor::CAN_PACKET_STATUS,
    21.0f,1.0f, 20000.0f, 0.005f,
    1.5f, 0.5f, 0.0f, 20000.0f, 20000.0f,
    OutputFilter);



std::array<MotorBase*, 4> chassis_drivemotors{&chassis_drivemotor1, &chassis_drivemotor2,
                                              &chassis_drivemotor3, &chassis_drivemotor4};
/*-------------------------------------fdcan2----------------------------------------*/



/*-------------------------------------fdcan3----------------------------------------*/



osThreadId_t can3Send_TaskHandle;

CanBus fdcan3_bus(hfdcan3);

C620Motor chassis_dirmotor1(&fdcan3_bus, 0x201, false, 0x200, false, 
    MotorBase::PIDMode::DEGREE, 3591.0f / 187.0f,20000.0f, 0.005f,
    16.0f, 5.0f, 0.0f, 20000.0f, 20000.0f,
    OutputFilter,
    5.0f);

C620Motor chassis_dirmotor2(&fdcan3_bus, 0x202, false, 0x200, false, 
    MotorBase::PIDMode::DEGREE, 3591.0f / 187.0f,20000.0f, 0.005f,
    16.0f, 5.0f, 0.0f, 20000.0f, 20000.0f,
    OutputFilter,
    5.0f);

C620Motor chassis_dirmotor3(&fdcan3_bus, 0x203, false, 0x200, false, 
    MotorBase::PIDMode::DEGREE, 3591.0f / 187.0f,20000.0f, 0.005f,
    16.0f, 5.0f, 0.0f, 20000.0f, 20000.0f,
    OutputFilter,
    5.0f);

C620Motor chassis_dirmotor4(&fdcan3_bus, 0x204, false, 0x200, false, 
    MotorBase::PIDMode::DEGREE, 3591.0f / 187.0f,20000.0f, 0.005f,
    16.0f, 5.0f, 0.0f, 20000.0f, 20000.0f,
    OutputFilter,
    5.0f);

std::array<MotorBase*, 4> chassis_dirmotors{&chassis_dirmotor1, &chassis_dirmotor2,
                                            &chassis_dirmotor3, &chassis_dirmotor4};

/*-------------------------------------fdcan3----------------------------------------*/



/*-------------------------------------usart----------------------------------------*/
osThreadId_t uart3Process_TaskHandle;

void onUart3RxCb(const uint8_t *data, size_t len, void *user);

DMA_BUFFER_ATTR static uint8_t uart3_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart3_tx_dma[64];


UartPort uart3_port(&huart3, uart3_rx_dma, sizeof(uart3_rx_dma),
                    uart3_tx_dma, sizeof(uart3_tx_dma), onUart3RxCb, nullptr);

osSemaphoreId_t uart3_rx_semaphore = NULL;

/*-------------------------------------usart----------------------------------------*/

uint8_t comServiceInit(){

    canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO0, 0);
    canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO1, 1);
    bspCanInit(&hfdcan1);

    canFilterInit(&hfdcan2, FDCAN_EXTENDED_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO0, 0);
    canFilterInit(&hfdcan2, FDCAN_EXTENDED_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO1, 1);
    bspCanInit(&hfdcan2);

    canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO0, 0);
    canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO1, 1);
    bspCanInit(&hfdcan3);

    fdcan1_bus.init();
    fdcan2_bus.init();
    fdcan3_bus.init();

    fdcan2_bus.registerDevice(&chassis_drivemotor1);
    fdcan2_bus.registerDevice(&chassis_drivemotor2);
    fdcan2_bus.registerDevice(&chassis_drivemotor3);
    fdcan2_bus.registerDevice(&chassis_drivemotor4);

    fdcan3_bus.registerDevice(&chassis_dirmotor1);
    fdcan3_bus.registerDevice(&chassis_dirmotor2);
    fdcan3_bus.registerDevice(&chassis_dirmotor3);
    fdcan3_bus.registerDevice(&chassis_dirmotor4);
    


    uart3_rx_semaphore = osSemaphoreNew(1, 0, NULL);
    uart3_port.startRxDmaIdle();




    return 0;
}


/*-------------------------------------usart----------------------------------------*/
void onUart3RxCb(const uint8_t *data, size_t len, void *user){
    (void)user;
    if(data != nullptr && len > 0 && uart3_rx_semaphore != nullptr){
        osSemaphoreRelease(uart3_rx_semaphore);
    }
}

void uart3RxProcessTask(void *argument){

    (void)argument;
    for(;;){
        osSemaphoreAcquire(uart3_rx_semaphore, osWaitForever);
        
        UartPort::Packet packet{};
        while(uart3_port.Read(packet)){
            uart3_port.write(packet.data, packet.len);
        }
    }
}

/*-------------------------------------usart----------------------------------------*/

/*-------------------------------------fdcan1----------------------------------------*/

void can1SendTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){        


        vTaskDelayUntil(&current, 1);
    }
}
/*-------------------------------------fdcan1----------------------------------------*/


/*-------------------------------------fdcan2----------------------------------------*/

void can2SendTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){        

        
        {
            chassis_drivemotor1.pidUpdate();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor1.getCommandID() << 8) |
                (chassis_drivemotor1.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor1.buildTx(pack.data);

            fdcan2_bus.addCanMsg(pack);
        }
        // vTaskDelayUntil(&current, 1);

        {
            chassis_drivemotor2.pidUpdate();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor2.getCommandID() << 8) |
                (chassis_drivemotor2.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor2.buildTx(pack.data);

            fdcan2_bus.addCanMsg(pack);
        }
        // vTaskDelayUntil(&current, 1);

        {
            chassis_drivemotor3.pidUpdate();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor3.getCommandID() << 8) |
                (chassis_drivemotor3.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor3.buildTx(pack.data);

            fdcan2_bus.addCanMsg(pack);
        }
        // vTaskDelayUntil(&current, 1);

        {
            chassis_drivemotor4.pidUpdate();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor4.getCommandID() << 8) |
                (chassis_drivemotor4.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor4.buildTx(pack.data);

            fdcan2_bus.addCanMsg(pack);
        }

        vTaskDelayUntil(&current, 1);
    }
}
/*-------------------------------------fdcan2----------------------------------------*/


/*-------------------------------------fdcan3----------------------------------------*/

void can3SendTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){
        chassis_dirmotor1.pidUpdate();
        chassis_dirmotor2.pidUpdate();
        chassis_dirmotor3.pidUpdate();
        chassis_dirmotor4.pidUpdate();

        uint8_t data[8] = {0};
        CanBus::ClassicPack pack = {};

        uint8_t len = 0U;

        uint32_t motor_ids[4] = {0, 0x202, 0 ,0};
        int16_t commands[4] = {0, static_cast<int16_t>(chassis_dirmotor1.cmdTrans()), 0, 0};

        packDJIMotorCanMsg(0x200, motor_ids, commands, 4U, data, len);

        pack.id = 0x200;
        pack.type = CanBus::Type::STANDARD;
        pack.dlc = FDCAN_DLC_BYTES_8;
        for(uint8_t i = 0; i < 8; ++i) pack.data[i] = data[i];
        fdcan3_bus.addCanMsg(pack);
            
        vTaskDelayUntil(&current, 1);
        
    }

}

/*-------------------------------------fdcan3----------------------------------------*/
