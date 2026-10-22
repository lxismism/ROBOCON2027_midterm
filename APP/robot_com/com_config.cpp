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

#include "memory_map.h"
#include "UartPort.hpp"
#include "WitMotionimu.hpp"

#include "Canbus.hpp"
#include "MotorBase.hpp"
#include "djiMotor.hpp"
#include "vescMotor.hpp"
#include "dmMotor.hpp"

#include "topics.hpp"
#include "topic_pool.h"
#include <array>
#include "chassis_solution.hpp"

#include "Remote_receiver.hpp"
/*-------------------------------------fdcan1----------------------------------------*/

osThreadId_t can1Send_TaskHandle;
CanBus fdcan1_bus(hfdcan1);

// 达妙 4310：Master ID = 0x11，CAN ID = 1，位置+速度模式。
DM4310Motor motor_dm(&fdcan1_bus, 0x11, 0x01,
    DM4310Motor::PosWithSpeed, 1.0f);

// 新增 2006 挂在 CAN1，电调编号 5，位置控制。
C610Motor elongate_motor(&fdcan1_bus, 0x207, false, 0x1FF, false,
    MotorBase::OutputType::DEGREE, 1.0f, 10000.0f, 0.01f,
    10.0f, 0.0f, 0.02f, 10000.0f, 500.0f,
    NONE, 0.1f,
    3.0f, 0.0f, 0.02f, 180.0f, 0.0f,
    NONE, 0.5f);

VESCMotor chassis_drivemotor1(&fdcan1_bus, 101, MotorBase::OutputType::SPEED, 
    VESCMotor::CAN_PACKET_SET_CURRENT, VESCMotor::CAN_PACKET_STATUS,
    21.0f, 1.0f, 20000.0f, 0.0f,
    7.0f, 5.0f, 0.17f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT, 0.1f);

VESCMotor chassis_drivemotor2(&fdcan1_bus, 102, MotorBase::OutputType::SPEED, 
    VESCMotor::CAN_PACKET_SET_CURRENT,VESCMotor::CAN_PACKET_STATUS,
    21.0f,1.0f, 20000.0f, 0.0f,
    7.0f, 5.0f, 0.17f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT, 0.1f);

VESCMotor chassis_drivemotor3(&fdcan1_bus, 103, MotorBase::OutputType::SPEED, 
    VESCMotor::CAN_PACKET_SET_CURRENT, VESCMotor::CAN_PACKET_STATUS,
    21.0f,1.0f, 20000.0f, 0.00f,
    7.0f, 5.0f, 0.17f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT, 0.1f);

VESCMotor chassis_drivemotor4(&fdcan1_bus, 104, MotorBase::OutputType::SPEED, 
    VESCMotor::CAN_PACKET_SET_CURRENT, VESCMotor::CAN_PACKET_STATUS,
    21.0f,1.0f, 20000.0f, 0.0f,
    7.0f, 5.0f, 0.17f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT, 0.1f);



std::array<MotorBase*, 4> chassis_drivemotors{&chassis_drivemotor1, &chassis_drivemotor2,
                                              &chassis_drivemotor3, &chassis_drivemotor4};



/*-------------------------------------fdcan1----------------------------------------*/



/*-------------------------------------fdcan2----------------------------------------*/
osThreadId_t can2Send_TaskHandle;
CanBus fdcan2_bus(hfdcan2);

// 新增两台 3508，电调编号 5、6；与原有编号 1~4 分组发送。
// PID 参数为调试初值，速度环电流限幅 2000 mA，位置环速度限幅 180 °/s。
C620Motor lifting_motor(&fdcan2_bus, 0x205, false, 0x1FF, false,
    MotorBase::OutputType::DEGREE, 20.0f, 20000.0f, 0.01f,
    20.0f, 0.0f, 0.0f, 20000.0f, 1000.0f,
    NONE, 0.1f,
    15.0f, 0.0f, 0.0f, 1000.0f, 0.0f,
    NONE, 0.5f);

C620Motor spin_motor(&fdcan2_bus, 0x206, false, 0x1FF, false,
    MotorBase::OutputType::DEGREE, 8.0f, 20000.0f, 0.01f,
    15.0f, 0.0f, 0.02f, 20000.0f, 1000.0f,
    NONE, 0.1f,
    6.0f, 0.0f, 0.02f, 1800.0f, 0.0f,
    NONE, 0.5f);

C620Motor chassis_dirmotor1(&fdcan2_bus, 0x201, false, 0x200, false, 
    MotorBase::OutputType::DEGREE, 8.0f,20000.0f, 0.01f,
    400.0f, 5000.0f, 0.0f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT | OutputFilter, 0.1f,
    20.0f, 0.0f, 0.0f, 6000.0f, 6000.0f,
    Derivative_On_Measurement, 1.0f);

C620Motor chassis_dirmotor2(&fdcan2_bus, 0x202, false, 0x200, false, 
    MotorBase::OutputType::DEGREE, 8.0f,20000.0f, 0.01f,
    400.0f, 5000.0f, 0.0f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT | OutputFilter, 0.1f,
    20.0f, 0.0f, 0.0f, 6000.0f, 6000.0f,
    Derivative_On_Measurement, 1.0f);

C620Motor chassis_dirmotor3(&fdcan2_bus, 0x203, false, 0x200, false, 
    MotorBase::OutputType::DEGREE, 8.0f,20000.0f, 0.01f,
    400.0f, 5000.0f, 0.0f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT | OutputFilter, 0.1f,
    20.0f, 0.0f, 0.0f, 6000.0f, 6000.0f,
    Derivative_On_Measurement, 1.0f);

C620Motor chassis_dirmotor4(&fdcan2_bus, 0x204, false, 0x200, false, 
    MotorBase::OutputType::DEGREE, 8.0f,20000.0f, 0.01f,
    400.0f, 5000.0f, 0.0f, 20000.0f, 20000.0f,
    Derivative_On_Measurement | INCREATEMENT_OF_OUT | OutputFilter, 0.1f,
    20.0f, 0.0f, 0.0f, 6000.0f, 6000.0f,
    Derivative_On_Measurement, 1.0f);

std::array<MotorBase*, 4> chassis_dirmotors{&chassis_dirmotor1, &chassis_dirmotor2,
                                            &chassis_dirmotor3, &chassis_dirmotor4};
                                            

/*-------------------------------------fdcan2----------------------------------------*/

osThreadId_t can3Send_TaskHandle;
CanBus fdcan3_bus(hfdcan3);

// CAN3 暂不挂接电机。



/*-------------------------------------fdcan3----------------------------------------*/

/*-------------------------------------usart2----------------------------------------*/
osThreadId_t uart2Process_TaskHandle;

void onUart2RxCb(const uint8_t *data, size_t len, void *user);

DMA_BUFFER_ATTR static uint8_t uart2_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart2_tx_dma[64];


UartPort uart2_port(&huart2, uart2_rx_dma, sizeof(uart2_rx_dma),
                    uart2_tx_dma, sizeof(uart2_tx_dma), onUart2RxCb, nullptr);

osSemaphoreId_t uart2_rx_semaphore = NULL;

WitMotionimu wit_imu{};

static TypedTopicPublisher<pub_imu_data> imu_data_pub("imu_data");
static pub_imu_data imu_data{};
/*-------------------------------------usart2----------------------------------------*/

/*-------------------------------------usart3----------------------------------------*/
osThreadId_t uart3Process_TaskHandle;

void onUart3RxCb(const uint8_t *data, size_t len, void *user);

DMA_BUFFER_ATTR static uint8_t uart3_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart3_tx_dma[64];


UartPort uart3_port(&huart3, uart3_rx_dma, sizeof(uart3_rx_dma),
                    uart3_tx_dma, sizeof(uart3_tx_dma), onUart3RxCb, nullptr);

osSemaphoreId_t uart3_rx_semaphore = NULL;


/*-------------------------------------usart3----------------------------------------*/

uint8_t comServiceInit(){

    canFilterInit(&hfdcan1, FDCAN_EXTENDED_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO0, 0);
    canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO1, 1);
    bspCanInit(&hfdcan1);

    canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO0, 0);
    canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO1, 1);
    bspCanInit(&hfdcan2);

    canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO0, 0);
    canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, 0, 0, FDCAN_FILTER_TO_RXFIFO1, 1);
    bspCanInit(&hfdcan3);

    fdcan1_bus.init();
    fdcan2_bus.init();
    fdcan3_bus.init();

    fdcan1_bus.registerDevice(&chassis_drivemotor1);
    fdcan1_bus.registerDevice(&chassis_drivemotor2);
    fdcan1_bus.registerDevice(&chassis_drivemotor3);
    fdcan1_bus.registerDevice(&chassis_drivemotor4);
    fdcan1_bus.registerDevice(&motor_dm);
    fdcan1_bus.registerDevice(&elongate_motor);

    fdcan2_bus.registerDevice(&chassis_dirmotor1);
    fdcan2_bus.registerDevice(&chassis_dirmotor2);
    fdcan2_bus.registerDevice(&chassis_dirmotor3);
    fdcan2_bus.registerDevice(&chassis_dirmotor4); 
    fdcan2_bus.registerDevice(&lifting_motor);
    fdcan2_bus.registerDevice(&spin_motor);
    
    uart2_rx_semaphore = osSemaphoreNew(1, 0, NULL);
    uart2_port.startRxDmaIdle();

    uart3_rx_semaphore = osSemaphoreNew(1, 0, NULL);
    uart3_port.startRxDmaIdle();




    return 0;
}



/*-------------------------------------usart2----------------------------------------*/
void onUart2RxCb(const uint8_t *data, size_t len, void *user){
    (void)user;
    if(data != nullptr && len > 0 && uart2_rx_semaphore != nullptr){
        osSemaphoreRelease(uart2_rx_semaphore);
    }
}

void uart2RxProcessTask(void *argument){

    (void)argument;
    for(;;){
        osSemaphoreAcquire(uart2_rx_semaphore, osWaitForever);
        
        UartPort::Packet packet{};
        while(uart2_port.Read(packet)){
            for(uint8_t i = 0; i < packet.len; ++i){
                uint8_t frame_type = wit_imu.processByte(packet.data[i]);
                if(frame_type == 0x53){
                    imu_data = wit_imu.getImuData();
                    imu_data_pub.Publish(imu_data);
                }
            }
        }
    }
}

/*-------------------------------------usart2----------------------------------------*/

/*-------------------------------------usart3----------------------------------------*/
void onUart3RxCb(const uint8_t *data, size_t len, void *user){
    (void)user;
    if(data != nullptr && len > 0 && uart3_rx_semaphore != nullptr){
        osSemaphoreRelease(uart3_rx_semaphore);
    }
}

static TypedTopicPublisher<Remote::State>
    remote_publisher(Remote::Remote_state_topic);

static_assert(
    sizeof(Remote::State) <= TOPICS_MAX_MESSAGE_SIZE,
    "Remote State exceeds topic capacity");

    // 远程状态回调函数
static void on_remote_state(const Remote::State& state, void* user)
{
    (void)user;

    // 与订阅端配套保护现有覆盖队列。
    taskENTER_CRITICAL();
    (void)remote_publisher.Publish(state);
    taskEXIT_CRITICAL();
}

    //创建一个遥控实例
static Remote::Remote_receiver remote_receiver(on_remote_state,
    nullptr,Remote::koffline_timeout_ms);


void uart3RxProcessTask(void *argument){

    (void)argument;
    uint32_t wait_ticks = pdMS_TO_TICKS(10U);

    if (wait_ticks == 0U) {
        wait_ticks = 1U;
    }

    for(;;){
        osSemaphoreAcquire(uart3_rx_semaphore, osWaitForever);
        
        UartPort::Packet packet{};
        remote_receiver.poll(HAL_GetTick());
        while(uart3_port.Read(packet)){
           remote_receiver.feed(packet.data, packet.len, HAL_GetTick());
        }
        remote_receiver.poll(HAL_GetTick());
    }
}

/*-------------------------------------usart3----------------------------------------*/

/*-------------------------------------fdcan1----------------------------------------*/

void can1SendTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){        

        
        {
            chassis_drivemotor1.update();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor1.getCommandID() << 8) |
                (chassis_drivemotor1.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor1.buildTx(pack.data);

            fdcan1_bus.addCanMsg(pack);
        }
        // vTaskDelayUntil(&current, 1);

        {
            chassis_drivemotor2.update();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor2.getCommandID() << 8) |
                (chassis_drivemotor2.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor2.buildTx(pack.data);

            fdcan1_bus.addCanMsg(pack);
        }
        // vTaskDelayUntil(&current, 1);

        {
            chassis_drivemotor3.update();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor3.getCommandID() << 8) |
                (chassis_drivemotor3.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor3.buildTx(pack.data);

            fdcan1_bus.addCanMsg(pack);
        }
        // vTaskDelayUntil(&current, 1);

        {
            chassis_drivemotor4.update();
            CanBus::ClassicPack pack = {};  
            pack.id = (chassis_drivemotor4.getCommandID() << 8) |
                (chassis_drivemotor4.getID() & 0xFFU);      
            pack.dlc = FDCAN_DLC_BYTES_4;
            pack.type = CanBus::Type::EXTENDED;

            chassis_drivemotor4.buildTx(pack.data);

            fdcan1_bus.addCanMsg(pack);
        }

        {
            CanBus::ClassicPack pack{};
            pack.id = motor_dm.getTxID();
            pack.type = CanBus::Type::STANDARD;
            pack.dlc = FDCAN_DLC_BYTES_8;

            if(motor_dm.buildTx(pack.data)){
                fdcan1_bus.addCanMsg(pack);
            }
        }

        {
            elongate_motor.update();

            uint32_t motor_ids[1] = {0x205};
            int16_t commands[1] = {
                lifting_motor.isOffline() ? static_cast<int16_t>(0) :
                    static_cast<int16_t>(elongate_motor.cmdTrans())
            };
            CanBus::ClassicPack pack{};
            pack.id = 0x1FF;
            pack.type = CanBus::Type::STANDARD;
            pack.dlc = FDCAN_DLC_BYTES_8;
            packDJIMotorCanMsg(pack.id, motor_ids, commands, 1U, pack.data);
            fdcan1_bus.addCanMsg(pack);
        }

        vTaskDelayUntil(&current, 1);
    }
}
/*-------------------------------------fdcan1----------------------------------------*/


/*-------------------------------------fdcan2----------------------------------------*/

void can2SendTask(void *argument){
    TickType_t current = xTaskGetTickCount();

    for(;;){        
        chassis_dirmotor1.update();
        chassis_dirmotor2.update();
        chassis_dirmotor3.update();
        chassis_dirmotor4.update();

        uint8_t data[8] = {0};
        CanBus::ClassicPack pack = {};

        uint32_t motor_ids[4] = {0x201, 0x202, 0x203 ,0x204};
        int16_t commands[4] = {
            static_cast<int16_t>(chassis_dirmotor1.cmdTrans()),
            static_cast<int16_t>(chassis_dirmotor2.cmdTrans()),
            static_cast<int16_t>(chassis_dirmotor3.cmdTrans()),
            static_cast<int16_t>(chassis_dirmotor4.cmdTrans())
        };

        packDJIMotorCanMsg(0x200, motor_ids, commands, 4U, data);

        pack.id = 0x200;
        pack.type = CanBus::Type::STANDARD;
        pack.dlc = FDCAN_DLC_BYTES_8;
        for(uint8_t i = 0; i < 8; ++i) pack.data[i] = data[i];
        fdcan2_bus.addCanMsg(pack);

        // 编号 5、6 使用 0x1FF，两个电流指令合在同一帧中。
        {
            lifting_motor.update();
            spin_motor.update();

            uint32_t motor_ids[2] = {0x205, 0x206};
            int16_t commands[2] = {
                lifting_motor.isOffline() ? static_cast<int16_t>(0) :
                    static_cast<int16_t>(lifting_motor.cmdTrans()),
                spin_motor.isOffline() ? static_cast<int16_t>(0) :
                    static_cast<int16_t>(spin_motor.cmdTrans())
            };
            CanBus::ClassicPack pack{};
            pack.id = 0x1FF;
            pack.type = CanBus::Type::STANDARD;
            pack.dlc = FDCAN_DLC_BYTES_8;
            packDJIMotorCanMsg(pack.id, motor_ids, commands, 2U, pack.data);
            fdcan2_bus.addCanMsg(pack);
        }

        vTaskDelayUntil(&current, 1);
    }
}
/*-------------------------------------fdcan2----------------------------------------*/


/*-------------------------------------fdcan3----------------------------------------*/

void can3SendTask(void *argument){
    (void)argument;
    TickType_t current = xTaskGetTickCount();

    for(;;){
        // CAN3 不挂电机，保留空任务供后续使用。
        vTaskDelayUntil(&current, 5);
        
    }

}

/*-------------------------------------fdcan3----------------------------------------*/
