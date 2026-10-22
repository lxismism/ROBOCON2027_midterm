/**
 * @file chassis_solution.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
    底盘解算的四个舵轮布局如下：
            1    0   

            2    3

 * @version 0.1
 * @date 2026-10-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include "gpio.h"

#include "MotorBase.hpp"
#include "CanBus.hpp"
#include "djiMotor.hpp"
#include "vescMotor.hpp"

#include "photogate.hpp"

#include "topic_pool.h"
#include "math_utils.hpp"
#include <array>

static constexpr uint8_t kWheelNum = 4U;        //轮数

static constexpr std::array<std::array<float, 2>, kWheelNum> edge_at_zeros{{
    {-135.0f, 45.0f},
    {-60.0f, 120.0f},
    {30.0f, -150.0f},
    {120.0f, -60.0f}
}};

static constexpr uint8_t homing_ways[kWheelNum]{0, 1, 0, 1};

extern PhotoGate chassis_photogate1;
extern PhotoGate chassis_photogate2;
extern PhotoGate chassis_photogate3;
extern PhotoGate chassis_photogate4;
extern std::array<PhotoGate*, kWheelNum> chassis_photogates;

struct Velocity_t{
    float dir;
    float speed;
};

class SteerWheel{
public:
    SteerWheel() = default;

    enum class HomingState : uint8_t{
        Find_First_Zero = 0,
        Find_Second_Zero,
        Find_Third_Zero,
        Check,
        Finish,
    };

    void init(C620Motor* dirmotor, VESCMotor* drivemotor, PhotoGate* photogate, 
              uint8_t homing_way, const std::array<float, 2>& edge_at_zero){
        dirmotor_ = dirmotor;
        drivemotor_ = drivemotor;
        photogate_ = photogate;
        homing_way_ = homing_way;
        edge_at_zero_ = edge_at_zero;
    }

    void setVelocity(Velocity_t target_velocity){

        updateWheelState();

        float deg_output = 0.0f;        //由于电机位置环的输入是累计的绝对角度，所以此处也必须用舵轮的累计角度计算，输入和输出必须匹配
        float degspeed_output = 0.0f;
        float diff = target_velocity.dir - current_velocity_.dir;


        if(diff > 90.0f){
            if(diff > 270.0f){
                target_velocity.dir -= 360.0f;

            } else{
                target_velocity.dir -= 180.0f;
                target_velocity.speed = -target_velocity.speed;
            }
        }
        else if(diff < -90.0f){
            if(diff < -270.0f){
                target_velocity.dir += 360.0f;
                
            } else{
                target_velocity.dir += 180.0f;
                target_velocity.speed = -target_velocity.speed;
            }
        }
        diff = target_velocity.dir - current_velocity_.dir;
        

        deg_output = (wheel_sum_deg_ + diff) * kWheelDirReduction;
        degspeed_output = (target_velocity.speed) * RAD_2_DEG * kWheelDriveReduction / kWheelRadmeter; 

        dirmotor_->setMotorDeg(deg_output);

        const float steer_err = deg_output - dirmotor_->getSumDeg();
        const float drive_scale = std::fmaxf(0.0f, std::cos(steer_err * DEG_2_RAD));
        drivemotor_->setMotorDegSpeed(degspeed_output * drive_scale);

    }

    void homing(void){
        if(hashoming_ || dirmotor_->isOffline()) return;

        ramp_deg_homing_ += (map_[homing_way_] * deg_homing_speed_);
        dirmotor_->setMotorDeg(ramp_deg_homing_);
        photogate_->update();
        
        switch (homing_state_) {
            case HomingState::Find_First_Zero :{
                if(photogate_->hasRisingEdge()){
                    first_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_];
                    homing_state_ = HomingState::Find_Second_Zero;
                } 
                else if(photogate_->hasFallingEdge()){
                    first_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_^1];        //^1的作用：0变1,1变0，在此处是为了切换数组变量，这种方式优雅一点
                    homing_state_ = HomingState::Find_Second_Zero;
                }
            }; break;

            case HomingState::Find_Second_Zero :{
                if(photogate_->hasRisingEdge()){
                    second_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_];
                    if(std::fabs(second_zero_ - first_zero_) > 170.0f){
                        homing_state_ = HomingState::Find_Third_Zero;
                    }       //这个判断是为了防止光电门在边沿附近抖动导致误触发判断，需要超过170度才能保证，它经过的边沿跟第一次经过的不是同一个，不然没意义啊。
                }
                else if(photogate_->hasFallingEdge()){
                    second_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_^1];
                    if(std::fabs(second_zero_ - first_zero_) > 170.0f){
                        homing_state_ = HomingState::Find_Third_Zero;
                    }       
                }
                

            }; break;

            case HomingState::Find_Third_Zero :{
                if(photogate_->hasRisingEdge()){
                    third_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_];
                    if(std::fabs(third_zero_ - second_zero_) > 170.0f){
                        homing_state_ = HomingState::Check;
                    }       //这个判断是为了防止光电门在边沿附近抖动导致误触发判断，需要超过170度才能保证，它经过的边沿跟第一次经过的不是同一次，不然没意义啊。
                }
                else if(photogate_->hasFallingEdge()){
                    third_zero_ = dirmotor_->getSumDeg() - edge_at_zero_[homing_way_^1];
                    if(std::fabs(third_zero_ - second_zero_) > 170.0f){
                        homing_state_ = HomingState::Check;
                    }       
                }
                
            }; break;

            case HomingState::Check :{
                const float first_offset_ = math_utils::WrapAngle(first_zero_, -180.0f, 180.0f);
                const float second_offset_ = math_utils::WrapAngle(second_zero_, -180.0f, 180.0f);
                const float third_offset_ = math_utils::WrapAngle(third_zero_, -180.0f, 180.0f);

                if(std::fabs(first_offset_ - second_offset_) < 5.0f &&
                   std::fabs(second_offset_ - third_offset_) < 5.0f){
                    deg_offset_ = (first_offset_ + second_offset_ + third_offset_) / 3.0f;
                    homing_state_ = HomingState::Finish;
                } else {
                    homing_state_ = HomingState::Find_First_Zero;
                }
            }; break;

            case HomingState::Finish :{
                hashoming_ = true;
            }; break;

        }
        
    }

    bool IsReady(void) { return hashoming_; }

private:
    static constexpr float kWheelDiameter = 0.104f;     //轮径
    static constexpr float kWheelRadmeter = kWheelDiameter / 2.0f;      //轮半径
    static constexpr float kWheelCirmeter = kPI * kWheelDiameter;        //轮周
    static constexpr float kWheelDriveReduction = 1.0f;     //电机输出轴->驱动轮的减速比
    static constexpr float kWheelDirReduction = 1.0f;       //电机输出轴->舵向轮的减速比

    C620Motor* dirmotor_{};
    VESCMotor* drivemotor_{};
    PhotoGate* photogate_{};

/*------------------------------homing----------------------------*/
    uint8_t homing_way_{0};  //默认逆时针转动来归零
    std::array<float, 2> map_{1.0f, -1.0f};


    std::array<float, 2> edge_at_zero_{-150.0f, 30.0f};     //这个数组存的，是两个cnc边沿，在极坐标中的真实位置，由且仅由安装位置决定。
                                                                    //编号为0的数组意为逆时针针转时，遇到的上升沿位置，这里根据0号轮填了个默认值。
    float deg_offset_{};
    float ramp_deg_homing_{};
    float deg_homing_speed_{0.1f};

    float first_zero_{};    
    float second_zero_{};
    float third_zero_{};

    HomingState homing_state_{};
    bool hashoming_{false};
/*------------------------------homing----------------------------*/

    Velocity_t current_velocity_{};
    Velocity_t last_velocity_{};
    float wheel_sum_deg_{0.0f};

    void updateWheelState(){

        wheel_sum_deg_ = dirmotor_->getSumDeg() / kWheelDirReduction;
        
        current_velocity_.dir = math_utils::WrapAngle(wheel_sum_deg_ - deg_offset_,
            -180.0f, 180.0f);
        current_velocity_.speed = drivemotor_->getDegSpeed() * DEG_2_RAD * kWheelRadmeter / kWheelDriveReduction;
        
    }

};

class SteerChassis{
    
    /*
        1    0   

        2    3
    */

public:
    SteerChassis(const std::array<C620Motor* , kWheelNum> &dirmotors,
                 const std::array<VESCMotor* , kWheelNum> &drivemotors){
        for(uint8_t i = 0; i < kWheelNum; ++i){
            steerwheels_[i].init(dirmotors[i], drivemotors[i], 
                chassis_photogates[i], homing_ways[i], edge_at_zeros[i]);
        }
    }

    void run(const pub_chassis_cmd &cmd){
        const float vx = cmd.linear_x_;
        const float vy = cmd.linear_y_;
        const float omega = cmd.omega_;

        velocitys_[0].speed = std::sqrt((vx - omega * ky) * (vx - omega * ky) + 
                                     (vy + omega * kx) * (vy + omega * kx));
        velocitys_[1].speed = -std::sqrt((vx - omega * ky) * (vx - omega * ky) + 
                                     (vy - omega * kx) * (vy - omega * kx));
        velocitys_[2].speed = std::sqrt((vx + omega * ky) * (vx + omega * ky) + 
                                     (vy - omega * kx) * (vy - omega * kx));
        velocitys_[3].speed = -std::sqrt((vx + omega * ky) * (vx + omega * ky) + 
                                     (vy + omega * kx) * (vy + omega * kx));
        desaturate();
        
        velocitys_[0].dir = std::atan2(vy + omega * kx , vx - omega * ky) * RAD_2_DEG;
        velocitys_[1].dir = std::atan2(vy - omega * kx , vx - omega * ky) * RAD_2_DEG;
        velocitys_[2].dir = std::atan2(vy - omega * kx , vx + omega * ky) * RAD_2_DEG;
        velocitys_[3].dir = std::atan2(vy + omega * kx , vx + omega * ky) * RAD_2_DEG;

        if(std::fabs(vx) < 0.01f &&
           std::fabs(vy) < 0.01f &&
           std::fabs(omega) < 0.01f){
            velocitys_[0].dir = 135.0f;
            velocitys_[1].dir = -135.0f;
            velocitys_[2].dir = -45.0f;
            velocitys_[3].dir = 45.0f;
        }

        for(uint8_t i = 0; i < kWheelNum; ++i){
            steerwheels_[i].setVelocity(velocitys_[i]);
        }

    }

    void homing(void){
        for(uint8_t i = 0; i < kWheelNum; ++i){
            steerwheels_[i].homing();
        }
    }

    bool HasHoming(void){
        return (steerwheels_[0].IsReady() &&
                steerwheels_[1].IsReady() &&
                steerwheels_[2].IsReady() &&
                steerwheels_[3].IsReady());
    }

private:

    static constexpr float kWheelbase = 1.0f;   //轴距
    static constexpr float kWheeltrack = 1.0f;  //轮距

    //ky,kx的值即便在同一个机器人上，也应该是可以不同的，其中，它的原点取决于你想让机器人自转时，绕机器人的哪个点旋转
    static constexpr float ky = kWheelbase / 2.0f;  //ky的含义是，点到x轴的距离
    static constexpr float kx = kWheeltrack / 2.0f; //kx的含义是，点到y轴的距离

    static constexpr float kWheelMaxSpeed = 2.5f;

    SteerWheel steerwheels_[kWheelNum]{};
    Velocity_t velocitys_[kWheelNum]{};

    void desaturate(){
        float max_speed = 0.0f;
        
        for(uint8_t i = 0; i < kWheelNum; ++i){
            const float speed = std::fabs(velocitys_[i].speed);
            if(speed > max_speed){
                max_speed = speed;
            }
        }

        if(max_speed < kWheelMaxSpeed){
            return;
        }

        const float scale = kWheelMaxSpeed / max_speed;
        for(uint8_t i = 0; i < kWheelNum; ++i){
            velocitys_[i].speed *= scale;
        }

    }

};