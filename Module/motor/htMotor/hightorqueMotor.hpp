/**
 * @file hightorqueMotor.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 高擎（Hightorque）关节模组 —— 8 字节经典 CAN 驱动（官方 ht_can 协议）
 * @version 0.2
 * @date 2026-11-09
 *
 * @copyright Copyright (c) 2026
 *
 * ================= 上一版为什么电机不转（问题定位） =================
 * 上一版下发的是 07 20 <pos:2> <vel:2> <tqe:2>，即把 CAN-FD 协议里的
 * 「寄存器 0x20 = 位置 / 0x21 = 速度 / 0x22 = 扭矩」直接搬到 8 字节经典帧上。
 * 但经典 CAN（8 字节）协议里，前两字节是**固定命令字**，不是子帧头：
 *     07 07  正常模式（位置+扭矩，或 速度+扭矩）
 *     07 35  协同模式（速度+扭矩+位置，注意字节顺序不一样）
 *     05 13  纯扭矩模式
 *     17 01  读状态（回 27 01 ...）
 *     01 00 00 停机
 * 07 20 不是合法命令，电机收到既不动作也不回包 → 串口里 rx 一直是 0；
 * 而驱动又要求「先收到反馈再发设定值」（sync 门控），于是永远发不出设定帧，
 * 电机自然一动不动。这就是截图里 HT NOFB sync=0 tx=183 rx=0 的原因。
 *
 * ================= 官方 ht_can 协议要点（8 字节经典帧） =================
 * 帧 ID：16 位 —— 高 8 位是源地址（bit15 = 1 表示要求对方回包），低 8 位是目的地址
 *     控制帧：0x0001（标准帧，低字节 = 目的地址）或 0x8001（扩展帧，带要求回包位）
 *     读状态：0x8001（扩展帧，必须带 bit15 电机才回包）
 *     回包帧：0x0100（高字节 = 电机 ID，低字节 = 目的地址 0）
 * 具体用哪个由 kCtrlUseExtended 决定，默认标准帧 0x0001（TR1 实测转得起来）。
 * 速度控制：07 07 00 80 <vel:2> <tqe:2>   —— pos = 0x8000 表示不限位置
 * 位置控制：07 07 <pos:2> 00 80 <tqe:2>   —— val = 0x8000 表示不限速度
 * 协同控制：07 35 <val:2> <tqe:2> <pos:2>
 * 读状态　：17 01            → 回 27 01 <pos:2> <vel:2> <tqe:2>
 * 停机　　：01 00 00
 * 所有 int16（位置/速度/扭矩）都是小端；一帧 8 字节，未用到的字节填 0
 * （官方 8 字节样例就是这么填的，CAN-FD 那边才用 0x50 占位）。
 *
 * 量纲（官方 3.6 / 3.7 节）：
 *     位置 int16：LSB = 0.0001 圈   → 1 圈 = 10000，量程 ±3.27 圈
 *     速度 int16：LSB = 0.00025 转/秒 → 1 转/秒 = 4000，量程 ±8.19 转/秒
 *     扭矩 int16：与型号强相关（斜率/偏置查手册 3.3 节），所以这里一律用
 *                 0x8000 = 「不限最大扭矩」，避免用错系数把扭矩限死或放大。
 *
 * 设计取舍：
 *   - 只发 8 字节经典帧，不需要改 fdcan.c / CanBus.cpp，不会影响 can3 上的
 *     达妙、大疆（C610/C620）经典帧；
 *   - 速度模式不需要反馈即可运行（TR1 工程实测可行），位置模式是绝对位置，
 *     没有反馈就不下发，避免上电直接往编码器零点甩。
 */

#pragma once

#include "Canbus.hpp"
#include "MotorBase.hpp"
#include <cstdint>

class HightorqueMotor : public CanDevice, public MotorBase {
public:
    enum class CtrlMode : uint8_t {
        Disable  = 0,
        Speed    = 1,   /* 07 07 00 80 <vel> <tqe>，闭环由电机内部做 */
        Position = 2,   /* 07 07 <pos> 00 80 <tqe>，绝对位置，需要先对齐 */
    };

    /* ------------------------- 协议常量 ------------------------- */
    static constexpr uint8_t  kPad        = 0x00;   /* 填充字节：官方 8 字节样例里未用到的字节都是 0 */
    static constexpr uint8_t  kCmdNormal  = 0x07;   /* 07 07 正常模式 */
    static constexpr uint8_t  kCmdCoop    = 0x35;   /* 07 35 协同模式（预留） */
    static constexpr uint8_t  kCmdStop    = 0x01;   /* 01 00 00 停机 */
    static constexpr uint8_t  kCmdRead    = 0x17;   /* 17 01 读 3×int16 */
    static constexpr uint8_t  kRegStatus  = 0x01;   /* 位置/速度/扭矩 起始寄存器 */
    static constexpr int16_t  kI16Nan     = static_cast<int16_t>(0x8000);  /* 不限制 */

    /* 量纲：counts per revolution / counts per revolution-per-second */
    static constexpr float kPosCountsPerRev = 10000.0f;   /* 0.0001 圈/count */
    static constexpr float kVelCountsPerRps = 4000.0f;    /* 0.00025 转/秒/count */
    static constexpr float kDeg2Pos         = kPosCountsPerRev / 360.0f;  /* 度 → count */
    static constexpr float kDegS2Vel        = kVelCountsPerRps / 360.0f;  /* 度/秒 → count */

    /* ------------------------- 帧调度（can3SendTask 5ms 调用） ------------------------- */
    static constexpr uint16_t kReadPeriodTicks = 20;   /* 每 20 拍读一次状态（约 100ms） */
    static constexpr uint16_t kStopFrameTicks  = 10;   /* 失能时补发 10 帧停机 */

    /* 控制帧用哪种 ID：
     *   false → 标准帧 0x001（低字节 = 目的地址 1，不带「要求回包」位）
     *           —— TR1 工程就是用它把电机转起来的（同一台车、同一条 can3、
     *              上面还有 0x06 的达妙，实测不冲突），所以默认用它，最稳。
     *           代价：电机不回包，串口里 rx 一直是 0（它本来就不回，不是故障；
     *              要看反馈靠下面那条独立的读帧）。
     *   true  → 扩展帧 0x8001（官方 ht_can 协议的写法，bit15=1 要求回包，
     *           电机每帧都会回 0x0100 的状态，tx/rx 都能在串口里对上）。
     * 读状态帧另外单独用 0x8001 扩展帧发，所以不管这里选哪个，都有机会拿到反馈。 */
    static constexpr bool kCtrlUseExtended = false;

    HightorqueMotor(CanBus *bus, uint32_t motor_id, float reduction = 1.0f)
        : CanDevice(bus, motor_id, false, motor_id, false)
    {
        (void)reduction;                     /* 0x20 位置本身就是输出轴位置，不参与换算 */
        output_type_ = OutputType::SPEED;
        max_cmd_     = 2800.0f;              /* int16 速度量程 ±8.19 转/秒 ≈ ±2949 度/秒 */
        mode_        = CtrlMode::Disable;    /* 默认不转，必须显式给目标 */
    }

    void init(void) {}

    /* ------------------------- 对外接口 ------------------------- */
    /* 目标输出轴角速度，单位 度/秒（正负均可）。不依赖反馈，上电即可运行 */
    void setTargetSpeed(float deg_per_sec) {
        mode_       = CtrlMode::Speed;
        ref_deg_s_  = deg_per_sec;
    }

    /* 目标位置：相对「上电时锁存的位置」的偏移，单位 度（需要反馈才能对齐） */
    void setTargetPosition(float deg) {
        mode_     = CtrlMode::Position;
        ref_deg_  = deg;
    }

    /* 加速度限幅，单位 度/秒²；<=0 表示关闭平滑（指令直接跳变） */
    void setAccelLimit(float deg_per_sec2) {
        accel_limit_ = (deg_per_sec2 > 0.0f) ? deg_per_sec2 : 0.0f;
    }

    void Disable(void) {
        mode_     = CtrlMode::Disable;
        cmd_deg_s_ = 0.0f;
        dis_tick_ = 0;
    }

    /* ------------------------- 周期调用（can3SendTask，5ms） ------------------------- */
    void update(void) {
        if (start_tick_ == 0U) {
            start_tick_ = HAL_GetTick();
        }

        if (mode_ == CtrlMode::Disable) {
            cmd_deg_s_ = 0.0f;
            return;
        }

        /* 用真实经过的时间做限幅，任务被长时间打断也不会一次跳太多 */
        const uint32_t now = HAL_GetTick();
        uint32_t dt_ms = now - last_cmd_tick_;
        last_cmd_tick_ = now;
        if (dt_ms == 0U) {
            dt_ms = 1U;
        } else if (dt_ms > 30U) {
            dt_ms = 30U;
        }

        if (mode_ == CtrlMode::Speed) {
            const float target = ref_deg_s_;
            if (accel_limit_ > 0.0f) {
                const float max_step = accel_limit_ * (0.001f * static_cast<float>(dt_ms));
                const float diff     = target - cmd_deg_s_;
                if (diff > max_step) {
                    cmd_deg_s_ += max_step;
                } else if (diff < -max_step) {
                    cmd_deg_s_ -= max_step;
                } else {
                    cmd_deg_s_ = target;
                }
            } else {
                cmd_deg_s_ = target;
            }
            /* 注意：Clamp 是返回值语义（不原地改），TR1 里那句写成了空操作 */
            cmd_deg_s_ = math_utils::Clamp(cmd_deg_s_, -max_cmd_, max_cmd_);
        } else {
            /* 位置模式：位置指令从当前位置斜坡推到目标，避免阶跃冲击 */
            if (!synced_) {
                return;                      /* 没反馈不动，等 onRx 对齐 */
            }
            const float target = sync_deg_ + ref_deg_;
            const float step   = ramp_deg_s_ * (0.001f * static_cast<float>(dt_ms));
            if (cmd_deg_ < target) {
                cmd_deg_ += step;
                if (cmd_deg_ > target) cmd_deg_ = target;
            } else if (cmd_deg_ > target) {
                cmd_deg_ -= step;
                if (cmd_deg_ < target) cmd_deg_ = target;
            }
        }
    }

    /* ------------------------- 组帧（每拍最多一帧 8 字节） ------------------------- */
    bool buildTx(uint8_t data[8]) override {
        for (uint8_t i = 0; i < 8U; ++i) {
            data[i] = kPad;
        }
        frame_id_  = id_;
        frame_ext_ = false;

        ++tick_;

        /* 1) 失能：补几帧停机，之后彻底静默（不刷总线） */
        if (mode_ == CtrlMode::Disable) {
            if (dis_tick_ < kStopFrameTicks) {
                ++dis_tick_;
                setCtrlAddr();
                data[0] = kCmdStop;
                data[1] = 0x00;
                data[2] = 0x00;
                return snapTx(data);
            }
            return false;
        }

        /* 2) 周期读状态：读帧必须带 bit15（0x8000|id）电机才会回包 */
        if ((tick_ % kReadPeriodTicks) == 0U) {
            data[0]    = kCmdRead;
            data[1]    = kRegStatus;
            frame_id_  = 0x8000U | (id_ & 0x7FU);
            frame_ext_ = true;
            return snapTx(data);
        }

        /* 3) 设定值帧：07 07 + pos + vel + tqe */
        setCtrlAddr();
        data[0] = kCmdNormal;
        data[1] = kCmdNormal;

        if (mode_ == CtrlMode::Position) {
            if (!synced_) {
                /* 没有反馈 = 不知道当前绝对位置，绝不能发位置指令（会直接甩到
                   编码器零点）；但也不能不发，否则电机会一直按上一条速度指令转。
                   这里发「位置不限 + 速度 0」，让它原地停住保持力矩。 */
                putI16(&data[2], kI16Nan);   /* 位置不限 */
                putI16(&data[4], 0);         /* 速度 0 */
            } else {
                putI16(&data[2], toI16(cmd_deg_ * kDeg2Pos));   /* 位置 */
                putI16(&data[4], kI16Nan);                      /* 速度不限 */
            }
        } else {
            putI16(&data[2], kI16Nan);                          /* 位置不限 */
            putI16(&data[4], toI16(cmd_deg_s_ * kDegS2Vel));    /* 速度 */
        }
        putI16(&data[6], kI16Nan);                              /* 最大扭矩不限 */

        return snapTx(data);
    }

    /* 本帧应该用哪个 ID / 帧类型（读帧要扩展帧 + 0x8001） */
    uint32_t     getFrameID(void)   const { return frame_id_; }
    CanBus::Type getFrameType(void) const {
        return frame_ext_ ? CanBus::Type::EXTENDED : CanBus::Type::STANDARD;
    }

    /* ------------------------- 反馈 ------------------------- */
    bool matchRx(const FDCAN_RxHeaderTypeDef &header) const override {
        const uint32_t id = header.Identifier;
        rx_hdr_id_  = id;
        rx_hdr_ext_ = (header.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;

        if ((id >> 8) == (id_ & 0x7FU)) {
            return true;                 /* 正常回包：0x0100 + 目的地址 */
        }
        if (id == id_) {
            return true;                 /* 有些固件直接用自己的 ID 回 */
        }
        if (id == (0x8000U | (id_ & 0x7FU))) {
            return true;
        }
        return false;
    }

    void onRx(const uint8_t data[8], uint8_t len) override {
        ++rx_frames_;
        last_rx_id_  = rx_hdr_id_;
        last_rx_ext_ = rx_hdr_ext_;
        rx_len_      = (len > 8U) ? 8U : len;
        for (uint8_t i = 0; i < rx_len_; ++i) {
            raw_rx_[i] = data[i];
        }
        last_rx_tick_ = HAL_GetTick();

        /* 回帧：27 01 <pos:2> <vel:2> <tqe:2> */
        if ((rx_len_ >= 8U) && (data[0] == 0x27U) && (data[1] == kRegStatus)) {
            const int16_t p = getI16(&data[2]);
            const int16_t v = getI16(&data[4]);
            const int16_t t = getI16(&data[6]);

            fb_pos_deg_  = (p == kI16Nan) ? 0.0f
                          : (static_cast<float>(p) / kPosCountsPerRev * 360.0f);
            fb_vel_deg_s = (v == kI16Nan) ? 0.0f
                          : (static_cast<float>(v) / kVelCountsPerRps * 360.0f);
            fb_tqe_raw_  = t;
            fb_valid_    = true;
            ++fb_frames_;

            sum_deg_   = fb_pos_deg_;
            deg_speed_ = fb_vel_deg_s;
            torque_    = static_cast<float>(t);

            if (!synced_) {              /* 第一次拿到反馈：锁存起点 */
                sync_deg_ = fb_pos_deg_;
                cmd_deg_  = sync_deg_;
                synced_   = true;
            }
        }
    }

    /* ------------------------- 调试取值 ------------------------- */
    uint32_t getTxFrames(void) const { return tx_frames_; }
    uint32_t getRxFrames(void) const { return rx_frames_; }
    uint32_t getFbFrames(void) const { return fb_frames_; }
    uint32_t getTxFail(void)   const { return tx_fail_; }
    void     notifyTxFail(void)     { ++tx_fail_; }

    bool  isSynced(void)       const { return synced_; }
    bool  isFeedbackValid(void) const { return fb_valid_; }
    CtrlMode getMode(void)     const { return mode_; }
    float getCmdDegSpeed(void) const { return cmd_deg_s_; }
    float getTargetDegSpeed(void) const { return ref_deg_s_; }
    float getTargetDeg(void)   const { return sync_deg_ + ref_deg_; }
    float getFbPosDeg(void)    const { return fb_pos_deg_; }
    float getFbVelDegS(void)   const { return fb_vel_deg_s; }
    int16_t getFbTqeRaw(void)  const { return fb_tqe_raw_; }

    const uint8_t *getLastTxData(void) const { return last_tx_; }
    uint32_t getLastTxId(void)         const { return last_tx_id_; }
    uint8_t  isLastTxExt(void)         const { return last_tx_ext_; }
    uint8_t  getLastTxLen(void)        const { return 8U; }
    const uint8_t *getLastRxData(void) const { return raw_rx_; }
    uint32_t getLastRxId(void)         const { return last_rx_id_; }
    uint8_t  isLastRxExt(void)         const { return last_rx_ext_; }
    uint8_t  getLastRxLen(void)        const { return rx_len_; }

    const char *getStateName(void) const {
        if (mode_ == CtrlMode::Disable) {
            return "DIS";
        }
        if (!fb_valid_) {
            return (mode_ == CtrlMode::Speed) ? "SPD/NOFB" : "POS/NOFB";
        }
        return (mode_ == CtrlMode::Speed) ? "SPD" : "POS";
    }

private:
    /* 控制帧/停帧的地址，见 kCtrlUseExtended 说明 */
    void setCtrlAddr(void) {
        if (kCtrlUseExtended) {
            frame_id_  = 0x8000U | (id_ & 0x7FU);
            frame_ext_ = true;
        } else {
            frame_id_  = id_;
            frame_ext_ = false;
        }
    }

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

    bool snapTx(const uint8_t *data) {
        for (uint8_t i = 0; i < 8U; ++i) {
            last_tx_[i] = data[i];
        }
        last_tx_id_  = frame_id_;
        last_tx_ext_ = frame_ext_ ? 1U : 0U;
        last_tx_tick_ = HAL_GetTick();
        ++tx_frames_;
        return true;
    }

    CtrlMode mode_{CtrlMode::Disable};

    float ref_deg_s_{0.0f};      /* 目标速度（度/秒） */
    float cmd_deg_s_{0.0f};      /* 斜坡后的下发速度（度/秒） */
    float ref_deg_{0.0f};        /* 位置模式：相对起点的偏移（度） */
    float cmd_deg_{0.0f};        /* 位置模式：斜坡后的下发角度（度） */
    float sync_deg_{0.0f};       /* 上电时锁存的实际位置（度） */
    float accel_limit_{1800.0f}; /* 度/秒²：0→240 度/秒约 0.13 秒 */
    float ramp_deg_s_{60.0f};    /* 位置模式斜坡（度/秒） */

    float   fb_pos_deg_{0.0f};
    float   fb_vel_deg_s{0.0f};
    int16_t fb_tqe_raw_{0};

    uint32_t frame_id_{0U};
    bool     frame_ext_{false};

    bool synced_{false};
    bool fb_valid_{false};

    uint32_t start_tick_{0U};
    uint32_t last_cmd_tick_{0U};
    uint32_t tick_{0U};
    uint32_t dis_tick_{kStopFrameTicks};

    uint8_t  raw_rx_[8]{};
    uint8_t  rx_len_{0U};
    uint32_t last_rx_id_{0U};
    uint8_t  last_rx_ext_{0U};
    uint32_t last_rx_tick_{0U};

    uint8_t  last_tx_[8]{};
    uint32_t last_tx_id_{0U};
    uint8_t  last_tx_ext_{0U};
    uint32_t last_tx_tick_{0U};

    uint32_t tx_frames_{0U};
    uint32_t rx_frames_{0U};
    uint32_t fb_frames_{0U};
    uint32_t tx_fail_{0U};

    /* matchRx 是 const，但要把帧头 ID 带给 onRx，只能 mutable */
    mutable uint32_t rx_hdr_id_{0U};
    mutable uint8_t  rx_hdr_ext_{0U};
};
