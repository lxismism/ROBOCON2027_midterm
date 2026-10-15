#pragma once

#include "Remote_receiver.hpp"

namespace Remote {

// 对应 State::ch[0]～ch[3]。
enum class Channel : uint8_t {
    LEFT_X = 0,
    LEFT_Y = 1,
    RIGHT_Y = 2,
    RIGHT_X = 3
};

// 0 -> -1，2048 -> 0，4095 -> 1。
inline int16_t normalize_channel(uint16_t raw)
{
    if (raw > 4095U) {
        raw = 4095U;
    }

    const int16_t offset = static_cast<int16_t>(raw) - 2048;
    return offset;
}

// 离线或通道编号无效时返回 0；调用前可用 is_fresh() 检查时效。
inline uint16_t get_channel_raw(const State &state, Channel channel)
{
    const uint8_t index = static_cast<uint8_t>(channel);
    if (!is_online(state) || index >= 4U) {
        return 0U;
    }
    return state.ch[index];
}

// 离线或通道编号无效时返回中立输出 0。
inline int16_t get_channel(const State &state, Channel channel)
{
    const uint8_t index = static_cast<uint8_t>(channel);
    if (!is_online(state) || index >= 4U) {
        return 0.0f;
    }
    return normalize_channel(state.ch[index]);
}

// 全部按键的位状态，离线时返回 0。
inline uint16_t get_keys(const State &state)
{
    return is_online(state) ? state.key : 0U;
}

// 普通按键表示按下；二值拨杆表示上位。
inline bool is_pressed(const State &state, key button)
{
    return (get_keys(state) & Musk(button)) != 0U;
}

} // namespace Remote
