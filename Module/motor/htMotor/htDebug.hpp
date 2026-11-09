/**
 * @file htDebug.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief can3 调试辅助：总线帧监听 + 串口调速命令解析
 * @version 0.1
 * @date 2026-11-09
 *
 * 两个类都不改 CanBus.cpp / fdcan.cpp：
 *
 * 1) CanTraceDevice —— 一个「全收」的 CanDevice，注册到 fdcan3 上。
 *    CanBus 收帧时会对每个注册设备调 matchRx/onRx，这里只是把最近几帧
 *    抄一份快照，由 debug1Task 每 500ms 打印。
 *    作用：不管电机回包的 ID 是什么、达妙/大疆的帧长什么样，串口里都能看见。
 *    对总线收发没有任何影响（只是多一次 memcpy）。
 *
 * 2) HtCmdParser —— 串口调速命令：
 *      m240   目标速度 240 度/秒（m0 停转，m-120 反转）
 *      p30    位置模式，相对上电位置 +30 度
 *      x      立即停转（等价 m0）
 *    UART3 和遥控接收机共用一个口，策略是「只要出现不属于命令的字节就整条丢弃」，
 *    遥控帧里凑出这种序列的概率极低。
 */

#pragma once

#include "Canbus.hpp"
#include <cstdint>
#include <cstdlib>

/* ---------------- fdcan3 总线监听 ---------------- */
class CanTraceDevice : public CanDevice {
public:
    static constexpr uint8_t kSlots = 8U;

    struct Slot {
        uint32_t id;
        uint32_t tick;
        uint8_t  ext;
        uint8_t  len;
        uint8_t  data[8];
    };

    explicit CanTraceDevice(CanBus *bus) : CanDevice(bus, 0, false, 0, false) {}

    /* 全收：只看帧头，不做过滤（onRx 拿不到帧头，所以在这里记一下） */
    bool matchRx(const FDCAN_RxHeaderTypeDef &header) const override {
        hdr_id_  = header.Identifier;
        hdr_ext_ = (header.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;
        return true;
    }

    void onRx(const uint8_t data[8], uint8_t len) override {
        const uint32_t n = cnt_;
        Slot &s = slots_[n % kSlots];
        s.id   = hdr_id_;
        s.ext  = hdr_ext_;
        s.len  = (len > 8U) ? 8U : len;
        s.tick = HAL_GetTick();
        for (uint8_t i = 0; i < s.len; ++i) {
            s.data[i] = data[i];
        }
        cnt_ = n + 1U;
    }

    uint32_t count(void) const { return cnt_; }

    /* back = 0 表示最后一帧，1 表示倒数第二帧…… */
    const Slot &get(uint32_t back) const {
        const uint32_t n = cnt_;
        uint32_t idx = (n + kSlots - 1U - back) % kSlots;
        if (back >= n) {
            idx = 0U;                         /* 还没收到那么多帧，给个安全值 */
        }
        return slots_[idx];
    }

private:
    mutable uint32_t hdr_id_{0U};
    mutable uint8_t  hdr_ext_{0U};
    Slot slots_[kSlots]{};
    volatile uint32_t cnt_{0U};
};

/* ---------------- 串口命令解析 ---------------- */
class HtCmdParser {
public:
    enum class Kind : uint8_t {
        NONE  = 0,
        SPEED = 1,   /* m<度/秒> */
        POS   = 2,   /* p<度>    */
        STOP  = 3,   /* x        */
        BAD   = 4,
    };

    static constexpr long kMaxAbs = 20000L;

    Kind feed(uint8_t b, float &out) {
        if (len_ == 0U) {
            if ((b == 'm') || (b == 'M')) {
                kind_ = Kind::SPEED;
                buf_[len_++] = 'm';
                return Kind::NONE;
            }
            if ((b == 'p') || (b == 'P')) {
                kind_ = Kind::POS;
                buf_[len_++] = 'p';
                return Kind::NONE;
            }
            if ((b == 'x') || (b == 'X')) {
                out = 0.0f;
                return Kind::STOP;
            }
            return Kind::NONE;                 /* 遥控帧字节，丢掉 */
        }

        if ((b == '\r') || (b == '\n')) {
            const Kind k = finish(out);
            len_ = 0U;
            return k;
        }

        if (((b >= '0') && (b <= '9')) || (b == '-') || (b == '+')) {
            if (len_ < (sizeof(buf_) - 1U)) {
                buf_[len_++] = static_cast<char>(b);
                return Kind::NONE;
            }
        }

        len_ = 0U;                             /* 混进噪声：整条作废 */
        return Kind::NONE;
    }

private:
    Kind finish(float &out) {
        buf_[len_] = '\0';
        if (len_ < 2U) {
            return Kind::BAD;                  /* 只有字母没有数字 */
        }

        char *end = nullptr;
        const long v = std::strtol(&buf_[1], &end, 10);
        if ((end == &buf_[1]) || (*end != '\0') || (v > kMaxAbs) || (v < -kMaxAbs)) {
            return Kind::BAD;
        }

        out = static_cast<float>(v);
        return kind_;
    }

    char    buf_[12]{};
    uint8_t len_{0U};
    Kind    kind_{Kind::NONE};
};
