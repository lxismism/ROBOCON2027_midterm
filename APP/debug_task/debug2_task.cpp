/**
 * @file debug_task2.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "debug1_task.h"
#include "debug2_task.h"
#include "topics.hpp"
#include "usart.h"
#include "com_config.h"
#include "chassis_solution.hpp"
#include "photogate.hpp"
#include <stdio.h>

osThreadId_t Debug2_TaskHandle;

extern VESCMotor chassis_drivemotor1;
extern VESCMotor chassis_drivemotor2;
extern VESCMotor chassis_drivemotor3;
extern VESCMotor chassis_drivemotor4;

extern C620Motor chassis_dirmotor1;
extern C620Motor chassis_dirmotor2;
extern C620Motor chassis_dirmotor3;
extern C620Motor chassis_dirmotor4;

extern PhotoGate chassis_photogate1;
extern PhotoGate chassis_photogate2;
extern PhotoGate chassis_photogate3;
extern PhotoGate chassis_photogate4;

uint32_t rising{};
uint32_t falling{};

void debug2Task(void *argument){
    TickType_t currentTime = xTaskGetTickCount();

    for(;;){
        
        chassis_photogate4.update();
        if(chassis_photogate4.hasRisingEdge()){
            rising++;
        }
        else if(chassis_photogate4.hasFallingEdge()){
            falling++;
        }

        vTaskDelayUntil(&currentTime, 20);
    }

}

void printDirDegSpeedCurse(){

    printf("%.3f,%.3f,%.3f,%.3f,"
           "%.3f,%.3f,%.3f,%.3f\r\n"
        ,chassis_dirmotor1.getRefDegSpeed()
        ,chassis_dirmotor2.getRefDegSpeed()
        ,chassis_dirmotor3.getRefDegSpeed()
        ,chassis_dirmotor4.getRefDegSpeed()

        ,chassis_dirmotor1.getDegSpeed()
        ,chassis_dirmotor2.getDegSpeed()
        ,chassis_dirmotor3.getDegSpeed()
        ,chassis_dirmotor4.getDegSpeed()
    );

}

void printDirDegCurse(){

    printf("%.3f,%.3f,%.3f,%.3f,"
           "%.3f,%.3f,%.3f,%.3f\r\n"
        ,chassis_dirmotor1.getRefDeg()
        ,chassis_dirmotor2.getRefDeg()
        ,chassis_dirmotor3.getRefDeg()
        ,chassis_dirmotor4.getRefDeg()

        ,chassis_dirmotor1.getSumDeg()
        ,chassis_dirmotor2.getSumDeg()
        ,chassis_dirmotor3.getSumDeg()
        ,chassis_dirmotor4.getSumDeg()
    );

}

void printDriveCurse(){

    printf("%.3f,%.3f,%.3f,%.3f,"
           "%.3f,%.3f,%.3f,%.3f\r\n"
        ,chassis_drivemotor1.getRefDegSpeed()
        ,chassis_drivemotor2.getRefDegSpeed()
        ,chassis_drivemotor3.getRefDegSpeed()
        ,chassis_drivemotor4.getRefDegSpeed()

        ,chassis_drivemotor1.getDegSpeed()
        ,chassis_drivemotor2.getDegSpeed()
        ,chassis_drivemotor3.getDegSpeed()
        ,chassis_drivemotor4.getDegSpeed()
    );

}