/**
 * @file double_buffer.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 双缓冲区
 * @version 0.1
 * @date 2026-09-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Algorithm{

/**
 * @brief 原始连续内存视图
 * 
 */
struct RawData{
    void* addr_ = nullptr;
    size_t size_ = 0;
};

/**
 * @brief 类型：双缓冲区
 * 
 */
class DoubleBuffer{
public:
    explicit DoubleBuffer(const RawData &raw_data) : SIZE(raw_data.size_ / 2){
        buffer_[0] = static_cast<uint8_t*>(raw_data.addr_);
        buffer_[1] = static_cast<uint8_t*>(raw_data.addr_) + SIZE;
    }
    
    uint8_t *ActiveBuffer() const { return buffer_[active_]; }
    uint8_t *PendingBuffer() const { return buffer_[1 - active_]; }
    size_t Size() const { return SIZE; }

    void Switch()
    {
        if(pending_valid_)
        {
            active_ ^= 1;
            active_len_ = pending_len_;
            pending_len_ = 0;
            pending_valid_ = false;
        }
    }

    bool HasPending() const { return pending_valid_; }
    
    bool FillPending(const uint8_t *data, size_t len)
    {
        if(pending_valid_ || data == nullptr || len > SIZE)
        {
            return false;
        }
        if(len != 0)
        {
            std::memcpy(PendingBuffer(), data, len);
        }
        pending_len_ = len;
        pending_valid_ = true;
        return true;
    }

    bool FillActive(const uint8_t *data, size_t len)
    {
        if(data == nullptr || len > SIZE)
        {
            return false;
        }

        if(len != 0)
        {
            std::memcpy(ActiveBuffer(), data, len);
        }
        active_len_ = len;
        return true;

    }

    size_t GetActiveLength() const { return active_len_; }
    


private:
    uint8_t *buffer_[2] = {nullptr,nullptr};
    const size_t SIZE;
    int active_ = 0;

    bool pending_valid_ = false ;
    size_t active_len_ = 0;
    size_t pending_len_ = 0;

};

}

