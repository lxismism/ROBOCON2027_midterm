/**
 * @file hightorqueMotor.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 高擎（Hightorque）关节模组 —— 8 字节经典 CAN 驱动（位置 / 运控 MIT 模式）
 * @version 0.1
 * @date 2026-11-08
 *
 * @copyright Copyright (c) 2026
 *
 * ======================== 为什么是 8 字节经典帧 ========================
 * 高擎协议的最小单位是「子帧」：子帧头(1B) + 起始寄存器(1B) + 数据。
 *   bit7~4 操作     0000=写   0001=读
 *   bit3~2 数据类型 00=int8   01=int16  10=int32  11=float
 *   bit1~0 数据个数 01=1个    10=2个    11=3个
 *
 * 用 int32 写「位置+速度」需要 13 字节（官方示例 01000a0a2000000080204e0000），
 * 只能走 CAN-FD。但用 int16 写「位置(0x20) + 速度(0x21) + 前馈扭矩(0x22)」
 * 这三个连续寄存器，正好 8 字节：
 *
 *     07 20 <pos:2> <vel:2> <tqe:2>
 *     07 = 写 int16 × 3，20 = 起始寄存器
 *
 * 整整一帧塞满经典 CAN 的 8 字节，一个填充字节都不需要。于是：
 *   - 不需要改 fdcan.c / CanBus.cpp / CanBus.hpp；
 *   - 不需要 CAN-FD，不会出现 FD 帧打断 C620/达妙经典帧的问题；
 *   - fdcan3 上三路电机可以照旧共存。
 *
 * ======================== 模式说明 ========================
 * 官方模式表里「位置模式」= 0x0A；「速度模式」其实就是位置模式下把位置
 * 填 0x8000(int16 的 NaN) 表示不限位置。所谓「运控模式(MIT)」也没有独立的
 * 模式值，它就是位置模式 + 写 Kp(0x23) / Kd(0x24) + 前馈扭矩(0x22)。
 * 因此位置模式和 MIT 模式下发的是同一条帧，区别只在 Kp/Kd 写不写。
 *
 * ======================== 量纲（需实测标定） ========================
 *   位置 int16：1 count = 0.0001 圈     → 1 圈 = 10000，量程 ±3.27 圈
 *   速度 int16：1 count = 0.00025 转/秒 → 1 转/秒 = 4000，量程 ±8.19 转/秒
 *   扭矩 int16：暂按 0.001 N·m/count（官方只给了 tqe=1000≈0.1N·m 的示例）
 *   0x8000 对 int16 表示「无限制 / 不关心」，本驱动所有中性值都用它。
 */

#pragma once

#include "Canbus.hpp"
#include "MotorBase.hpp"
#include <cstdint>
#include <cstring>

class HightorqueMotor : public CanDevice, public MotorBase {
public:
    enum class CtrlMode : uint8_t {
        Disable  = 0,
        Position = 1,   /* 位置模式（固件内部位置环） */
        Mit      = 2,   /* 运控模式：位置 + 速度 + 前馈扭矩 + 自填 Kp/Kd */
    };

    enum class State : uint8_t {
        WaitSync = 0,   /* 上电等第一帧反馈，拿当前位置当起点 */
        Run      = 1,   /* 正常跟随 */
        Disabled = 2,
    };

    /* ------------------------- 协议常量 ------------------------- */
    static constexpr uint8_t  kPad          = 0x50;  /* NOP 填充字节 */
    static constexpr uint8_t  kRegMode      = 0x00;  /* 电机模式 R/W */
    static constexpr uint8_t  kRegPos       = 0x20;  /* 位置指令 R/W */
    static constexpr uint8_t  kRegVel       = 0x21;  /* 速度命令 R/W（紧随 0x20） */
    static constexpr uint8_t  kRegTqe       = 0x22;  /* 前馈扭矩 R/W（紧随 0x21） */
    static constexpr uint8_t  kRegKp        = 0x23;  /* Kp R/W（float） */
    static constexpr uint8_t  kRegKd        = 0x24;  /* Kd R/W（float） */
    static constexpr uint8_t  kRegTqeLimit  = 0x25;  /* 最大扭矩 R/W（float） */
    static constexpr uint8_t  kRegFbPos     = 0x01;  /* 位置反馈 R（0x01/0x02/0x03 连续） */
    static constexpr uint8_t  kModeStop     = 0x00;  /* 停止 + 清错误 */
    static constexpr uint8_t  kModePosition = 0x0A;  /* 位置模式（10） */

    static constexpr int16_t  kI16Nan       = static_cast<int16_t>(0x8000);

    /* 子帧头：写 int16 × N */
    static constexpr uint8_t  kCmdWrI16x1   = 0x05;
    static constexpr uint8_t  kCmdWrI16x3   = 0x07;
    /* 子帧头：写 float × 1 */
    static constexpr uint8_t  kCmdWrF32x1   = 0x0D;
    /* 子帧头：写 int8 × 1（模式寄存器用） */
    static constexpr uint8_t  kCmdWrI8x1    = 0x01;
    /* 子帧头：读 int16 × 3 */
    static constexpr uint8_t  kCmdRdI16x3   = 0x17;

    /* ------------------------- 帧调度（1kHz 调用） ------------------------- */
    static constexpr uint16_t kModePeriodTicks = 200;   /* 模式帧 200ms 一次 */
    static constexpr uint16_t kModeBootTicks   = 2000;  /* 只在开机 2s 内补发 */
    static constexpr uint16_t kReadPeriodTicks = 20;    /* 读状态 20ms 一次 */
    static constexpr uint16_t kStopFrameTicks  = 10;    /* 失能时补发 10 帧停止 */

    /* ------------------------- 量纲换算 ------------------------- */
    static constexpr float kPosLsbRev = 0.0001f;   /* 位置：圈/count */
    static constexpr float kVelLsbRps = 0.00025f;  /* 速度：转每秒/count */
    static constexpr float kTqeLsbNm  = 0.001f;    /* 扭矩：N·m/count（待标定） */
    static constexpr float kDeg2Pos   = 1.0f / (kPosLsbRev * 360.0f);  /* ≈27.78 */
    static constexpr float kDeg2Vel   = 1.0f / (kVelLsbRps * 360.0f);  /* ≈11.11 */
    static constexpr float kNm2Tqe    = 1.0f / kTqeLsbNm;              /* 1000 */

    HightorqueMotor(CanBus *bus, uint32_t motor_id, float reduction = 1.0f)
        : CanDevice(bus, motor_id, false, motor_id, false)
    {
        (void)reduction;                 /* 0x20 寄存器本身就是输出轴位置，不参与换算 */
        output_type_ = OutputType::DEGREE;
        mode_  = CtrlMode::Disable;      /* 默认不发帧，必须显式设置目标才启动 */
        state_ = State::Disabled;
    }

    void init(void) {}

    /* ------------------------- 对外接口 ------------------------- */
    /* 位置模式：目标为输出轴角度（度），正负均可 */
    void setTargetPosition(float deg) {
        ref_deg_ = deg;
        mode_    = CtrlMode::Position;
        if (state_ == State::Disabled) state_ = State::WaitSync;
    }

    /* 运控（MIT）模式：需要同时给出 Kp/Kd，扭矩前馈用 setFeedForwardTorque 设 */
    void setTargetPositionMit(float deg) {
        ref_deg_ = deg;
        mode_    = CtrlMode::Mit;
        if (state_ == State::Disabled) state_ = State::WaitSync;
    }

    void setMitGains(float kp, float kd) { kp_ = kp; kd_ = kd; }

    /* 前馈扭矩（仅 MIT 模式生效），单位 N·m */
    void setFeedForwardTorque(float nm) { tqe_nm_ = nm; }

    /* 速度限幅：>0 写进 0x21，<=0 写 NaN（让固件自己决定） */
    void setVelLimitDegS(float deg_s) { vel_limit_deg_s_ = deg_s; }

    /* 指令斜坡，避免阶跃把电机甩出去 */
    void setRampDegPerSec(float deg_s) { ramp_deg_s_ = (deg_s > 1.0f) ? deg_s : 1.0f; }

    /* 最大扭矩限幅，>0 才下发 0x25，<=0 不下发 */
    void setTorqueLimitNm(float nm) { torque_limit_nm_ = nm; }

    void setSyncTimeoutMs(uint32_t ms) { sync_timeout_ms_ = ms; }

    /* 收不到反馈时是否允许强行按「当前位置=0」运行。
     * 默认 false：位置模式是绝对位置，不知道当前值就下发会直接跳到编码器零点，
     * 台架上可能猛甩。没有反馈就别动，先把 ID / 接线查清楚。 */
    void setAllowBlindStart(bool en) { allow_blind_ = en; }

    void Disable(void) {
        mode_    = CtrlMode::Disable;
        state_   = State::Disabled;
        dis_tick_ = 0;
    }

    /* ------------------------- 1kHz 周期调用 ------------------------- */
    void update(void) {
        if (start_tick_ == 0U) start_tick_ = HAL_GetTick();

        if (mode_ == CtrlMode::Disable) {
            state_ = State::Disabled;
            return;
        }

        /* 上电先对齐：位置模式是绝对位置，不知道当前值就下发会直接跳到编码器零点 */
        if (!synced_) {
            if (fb_valid_) {
                sync_deg_ = fb_pos_deg_;
                cmd_deg_  = sync_deg_;
                synced_   = true;
                state_    = State::Run;
            } else if ((HAL_GetTick() - start_tick_) > sync_timeout_ms_) {
                timeout_ = true;
                if (allow_blind_) {
                    /* 强行把当前位置当 0：会有一次未知大小的初始跳变，仅供台架调试 */
                    sync_deg_ = 0.0f;
                    cmd_deg_  = 0.0f;
                    synced_   = true;
                    state_    = State::Run;
                }
                /* 否则保持 WaitSync：不发设定值，电机保持不动 */
            }
            return;
        }

        /* 斜坡跟随 */
        const float target = sync_deg_ + ref_deg_;
        const float step   = ramp_deg_s_ * 0.001f;   /* 1ms 一拍 */
        if (cmd_deg_ < target) {
            cmd_deg_ += step;
            if (cmd_deg_ > target) cmd_deg_ = target;
        } else if (cmd_deg_ > target) {
            cmd_deg_ -= step;
            if (cmd_deg_ < target) cmd_deg_ = target;
        }
    }

    /* ------------------------- 组帧（每拍只出一帧 8B） ------------------------- */
    bool buildTx(uint8_t data[8]) override {
        for (uint8_t i = 0; i < 8U; ++i) data[i] = kPad;   /* 帧内其余字节一律 0x50 */

        ++tick_;

        if (mode_ == CtrlMode::Disable) {
            if (dis_tick_ < kStopFrameTicks) {             /* 补几帧「停止」再静默 */
                ++dis_tick_;
                data[0] = kCmdWrI8x1;
                data[1] = kRegMode;
                data[2] = kModeStop;
                ++tx_frames_;
                return true;
            }
            return false;
        }

        /* 1) 模式帧：开机 2s 内每 200ms 补一次，之后不再发（写模式可能复位目标） */
        if ((tick_ % kModePeriodTicks) == 1U && tick_ < kModeBootTicks) {
            data[0] = kCmdWrI8x1;
            data[1] = kRegMode;
            data[2] = kModePosition;
            ++tx_frames_;
            return true;
        }

        /* 2) 参数帧：上电最初几拍各写一次 */
        if (tick_ == 2U && mode_ == CtrlMode::Mit) {
            data[0] = kCmdWrF32x1; data[1] = kRegKp;
            std::memcpy(&data[2], &kp_, 4U);
            ++tx_frames_;
            return true;
        }
        if (tick_ == 3U && mode_ == CtrlMode::Mit) {
            data[0] = kCmdWrF32x1; data[1] = kRegKd;
            std::memcpy(&data[2], &kd_, 4U);
            ++tx_frames_;
            return true;
        }
        if (tick_ == 4U && torque_limit_nm_ > 0.0f) {
            data[0] = kCmdWrF32x1; data[1] = kRegTqeLimit;
            std::memcpy(&data[2], &torque_limit_nm_, 4U);
            ++tx_frames_;
            return true;
        }

        /* 3) 读状态：0x01/0x02/0x03 三个只读寄存器，回帧 27 01 + 3×int16 */
        if ((tick_ % kReadPeriodTicks) == 0U) {
            data[0] = kCmdRdI16x3;
            data[1] = kRegFbPos;
            ++tx_frames_;
            return true;
        }

        /* 4) 还没对齐就先别发设定值，否则会跳到编码器零点 */
        if (!synced_) return false;

        /* 5) 设定值帧：07 20 <pos> <vel> <tqe> */
        int16_t vel_raw = kI16Nan;
        if (vel_limit_deg_s_ > 0.0f) {
            vel_raw = toI16(vel_limit_deg_s_ * kDeg2Vel);
        }
        int16_t tqe_raw = kI16Nan;
        if (mode_ == CtrlMode::Mit) {
            tqe_raw = toI16(tqe_nm_ * kNm2Tqe);           /* 0 时也要下发 0，不能用 NaN */
        }

        data[0] = kCmdWrI16x3;
        data[1] = kRegPos;
        putI16(&data[2], toI16(cmd_deg_ * kDeg2Pos));
        putI16(&data[4], vel_raw);
        putI16(&data[6], tqe_raw);
        ++tx_frames_;
        return true;
    }

    /* ------------------------- 反馈解析 ------------------------- */
    void onRx(const uint8_t data[8], uint8_t len) override {
        ++rx_frames_;
        rx_len_ = len;

        /* 回帧格式：27 01 <pos:2> <vel:2> <tqe:2>，共 8 字节 */
        if (len >= 8U && data[0] == 0x27U && data[1] == kRegFbPos) {
            const int16_t p = getI16(&data[2]);
            const int16_t v = getI16(&data[4]);
            const int16_t t = getI16(&data[6]);

            fb_pos_deg_ = (p == kI16Nan) ? 0.0f : (static_cast<float>(p) * kPosLsbRev * 360.0f);
            fb_vel_deg_ = (v == kI16Nan) ? 0.0f : (static_cast<float>(v) * kVelLsbRps * 360.0f);
            fb_tqe_nm_  = (t == kI16Nan) ? 0.0f : (static_cast<float>(t) * kTqeLsbNm);
            fb_valid_   = true;
            fb_frames_++;

            /* 同步给 MotorBase，方便上层用 sum_deg_ / deg_speed_ */
            sum_deg_   = fb_pos_deg_;
            deg_speed_ = fb_vel_deg_;
            torque_    = fb_tqe_nm_;
        }
    }

    /* ------------------------- 调试用取值 ------------------------- */
    uint32_t getTxFrames(void) const { return tx_frames_; }
    uint32_t getRxFrames(void) const { return rx_frames_; }
    uint32_t getFbFrames(void) const { return fb_frames_; }
    State    getState(void)    const { return state_; }
    bool     isSynced(void)    const { return synced_; }
    bool     isSyncTimeout(void) const { return timeout_; }
    float    getFbPosDeg(void)  const { return fb_pos_deg_; }
    float    getFbVelDeg(void)  const { return fb_vel_deg_; }
    float    getCmdDeg(void)    const { return cmd_deg_; }
    float    getSyncDeg(void)   const { return sync_deg_; }
    float    getTargetDeg(void) const { return sync_deg_ + ref_deg_; }
    float    getErrDeg(void)    const { return fb_valid_ ? (getTargetDeg() - fb_pos_deg_) : 0.0f; }
    CtrlMode getMode(void)      const { return mode_; }

    const char *getStateName(void) const {
        if (timeout_ && !synced_) return "NOFB";   /* 3s 没收到反馈，检查 ID / 接线 */
        switch (state_) {
            case State::WaitSync: return "SYNC";
            case State::Run:      return "RUN";
            default:              return "DIS";
        }
    }

private:
    /* 四舍五入 + 限幅，永不产生 0x8000（那是 NaN） */
    static int16_t toI16(float v) {
        if (v >= 32767.0f)  return 32767;
        if (v <= -32767.0f) return -32767;
        return static_cast<int16_t>(v >= 0.0f ? (v + 0.5f) : (v - 0.5f));
    }
    static void putI16(uint8_t *p, int16_t v) {
        p[0] = static_cast<uint8_t>(v & 0xFF);
        p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    }
    static int16_t getI16(const uint8_t *p) {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                    (static_cast<uint16_t>(p[1]) << 8));
    }

    CtrlMode mode_{CtrlMode::Disable};
    State    state_{State::Disabled};

    float ref_deg_{0.0f};        /* 相对起点的目标偏移（度） */
    float cmd_deg_{0.0f};        /* 斜坡后的下发角度（度，绝对值） */
    float sync_deg_{0.0f};       /* 上电时锁存的实际位置（度） */

    float vel_limit_deg_s_{0.0f};
    float ramp_deg_s_{60.0f};
    float kp_{5.0f};
    float kd_{0.1f};
    float tqe_nm_{0.0f};
    float torque_limit_nm_{0.0f};

    float fb_pos_deg_{0.0f};
    float fb_vel_deg_{0.0f};
    float fb_tqe_nm_{0.0f};

    bool     synced_{false};
    bool     fb_valid_{false};
    bool     timeout_{false};
    bool     allow_blind_{false};

    uint32_t start_tick_{0U};
    uint32_t sync_timeout_ms_{3000U};
    uint32_t tick_{0U};
    uint32_t dis_tick_{kStopFrameTicks};   /* 上电不主动发停止帧，只有显式 Disable 才发 */
    uint8_t  rx_len_{0U};

    uint32_t tx_frames_{0U};
    uint32_t rx_frames_{0U};
    uint32_t fb_frames_{0U};
};
