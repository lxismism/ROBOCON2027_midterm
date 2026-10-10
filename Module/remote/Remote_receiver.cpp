#include "Remote_receiver.hpp"
#include <cstdint>
#include <cstring>
#include "math_utils.hpp"

namespace Remote{  
    Remote_receiver::Remote_receiver(
       State_callback callback,
       void *user,
       uint32_t timeout_ms
    ):callback_(callback),callback_user_(user),timeout_ms_(timeout_ms==0U?
        koffline_timeout_ms:timeout_ms){}


    uint16_t Remote_receiver::read_uint16(const uint8_t* data){
        return static_cast<uint16_t>(static_cast<uint16_t>(data[0])| data[1]<<8U);
    }

    //crc计算
    uint8_t Remote_receiver::crc8(
        const uint8_t* data,
        size_t len)
    {
        uint8_t crc = 0U;

        for (size_t i = 0U; i < len; ++i) {
            crc ^= data[i];

            for (uint8_t bit = 0U; bit < 8U; ++bit) {
                if ((crc & 0x80U) != 0U) {
                    crc = static_cast<uint8_t>(
                        (static_cast<uint16_t>(crc) << 1U) ^ 0xD5U);
                } else {
                    crc = static_cast<uint8_t>(
                        static_cast<uint16_t>(crc) << 1U);
                }
            }
        }

        return crc;
    }

    void Remote_receiver::notify_callback(){
        if(callback_ != nullptr){
            callback_(state_,callback_user_);
        }
    }

    void Remote_receiver::drop_front(){
       if(used_==0U){
        return;
       }

         --used_;

        if(used_>0U){
            std::memmove(frame_,frame_+1,used_);
        }
    }

    //对齐AA 55 帧头
    //只有一个AA的时候会等待
    void Remote_receiver:: align_header(){
        while(used_>=2U){
            if(frame_[0]==0xAA && frame_[1]==0x55){
                return;
            }

            drop_front();
        }

       if (used_ == 1U && frame_[0] != 0xAAU) {
            drop_front();
        }
    }

    bool Remote_receiver::decode_frame(uint32_t arrive_ms){
        if(frame_[15]!=0xDE)
        {
            return false;
        }

        //crc校验
        if(crc8(frame_+2, 12)!= frame_[14]){
            return false;
        }

        State next{};

        // 前两路保持反向，同时保持 2048 中点和两个端点。
        // 中点两侧跨度不同，分段缩放并四舍五入到原始通道精度。
        const auto reverse_channel = [](uint16_t raw) -> uint16_t {
            if (raw > 4095U) {
                return raw; // 留给下方的范围检查拒绝非法帧。
            }
            if (raw <= 2048U) {
                return static_cast<uint16_t>(
                    2048U + ((2048U - raw) * 2047U + 1024U) / 2048U);
            }
            return static_cast<uint16_t>(
                2048U - ((raw - 2048U) * 2048U + 1023U) / 2047U);
        };

        next.ch[0] = reverse_channel(read_uint16(frame_+2));
        next.ch[1] = reverse_channel(read_uint16(frame_+4));
        next.ch[2] = read_uint16(frame_+6);

        if(read_uint16(frame_+8)<1824)
        {
            next.ch[3] = read_uint16(frame_+8)*(2048/1600)-(2048/8);
        }else{
            next.ch[3] = read_uint16(frame_+8)+224;

        }
       


        next.key = read_uint16(frame_+10);
        next.page = frame_[12];
        next.display_color = frame_[13];

        for(int i = 0; i < 4; i++){
            if(next.ch[i]>4095){
             return false;
            }
        }

        //发送的页面
        if((next.page& 0x0F) > 3U){
            return false;
        }

        next.online=1u;
        next.last_rx_ms=arrive_ms;

        state_=next;

        notify_callback();
        return true;
    }


    void Remote_receiver::feed(const uint8_t *data, size_t len,uint32_t arrive_ms){
        if(data == nullptr || len == 0){
            return;
        }

        for(size_t i = 0; i < len; i++){
            frame_[used_++] = data[i];

            align_header();

            if(used_ < kFrameSize){
                continue;
            }

            if(decode_frame(arrive_ms))
            {
                used_ = 0U;
            }else
            {
                drop_front();
                align_header();
            }
        }
    }

    void Remote_receiver::set_offline(){
       const bool was_online = state_.online!=0;

       //丢出没有完成的帧
       used_ = 0U;

       state_.online=0u;

       //保留最后一次接收时间
       //online=0时，上层不能使用数据
       if(was_online){
        notify_callback();
        }
    }

    void Remote_receiver::poll(uint32_t now_ms) {
        if (state_.online == 0U) {
            return;
        }

        if (static_cast<uint32_t>(
                now_ms - state_.last_rx_ms) >= timeout_ms_)
        {
            set_offline();
        }
    }

    void Remote_receiver::break_stream(){
        set_offline();
    }
}
