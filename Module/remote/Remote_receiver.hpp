#pragma once

#include <cstdint>
#include <cstddef>


namespace Remote{
    constexpr const char* Remote_state_topic = "remote/state";
    constexpr uint32_t koffline_timeout_ms = 350U;


    //3轴拨杆的位置枚举
    enum class Switch_3_position: uint8_t{
        up =0,
        middle = 1,
        down = 2,
        invalid = 0xFF
    };

    //按键的状态枚举
    enum class key: uint16_t{
        //前面的按键
        Lt = 0x8000,   //左下
        Lb = 0x4000,   //左上
        Rb = 0x2000,   //右下    
        Rt = 0x1000,   //右上

        //中间的十字键
        Up =0x0800,   //上
        Left = 0x0400, //下
        Right = 0x0200, //左
        Down = 0x0100, //右

        //二值摇杆的按键，下为0，上为1
        Sw1= 0x0080,
        Sw2 = 0x0040,
        Sw3 = 0x0020,
        Sw4 = 0x0010,

        //三轴拨杆的按键
        //左拨杆的按键
        SwL_H = 0x0008,
        SwL_L = 0x0004,

        //右拨杆的按键
        SwR_H = 0x0002,
        SwR_L = 0x0001,
    };

    constexpr uint16_t Musk(key key)
    {
        return static_cast<uint16_t>(key);
    }


    struct State{
        uint16_t ch[4]{};//摇杆的四个通道
        uint16_t key{};//按键的状态

        uint8_t page{};//当前的按键页码
        uint8_t display_color{};//当前的显示颜色
        uint8_t online{};//当前的在线状态

         uint32_t last_rx_ms{};//上一次接收的时间戳
    };


    static_assert(sizeof(State) <=32, "Remote::State size must ");

    //判断是否在线,只是检查状态值里面的online字段是否为0,而不是检查是否超时
    //给跨任务用的
    inline bool is_online(const State &state)  { 
        return state.online != 0U; 
    }

    //判断是否新鲜,即是否在超时时间内接收到了新的数据
    inline bool is_fresh(const State &state,uint32_t now_ms, 
                        uint32_t timeout_ms=koffline_timeout_ms) 
                         {
                            return is_online(state)&&static_cast<uint32_t>(now_ms-state.last_rx_ms)<timeout_ms;
                         }   

    inline uint8_t get_page(const State &state)  { 
        return static_cast<uint8_t>(state.page&0x0FU); 
    }

    //3拨杆的位置
    inline Switch_3_position get_switch_3_position(uint8_t raw ){ 
       switch (raw & 0x03U) {
        case 0U:
            return Switch_3_position::middle;
        
        case 1U:
            return Switch_3_position::up;

        case 2U:
            return Switch_3_position::down;

        default:
            return Switch_3_position::invalid;
       }
    }

    inline Switch_3_position get_Swl_position(const State &state) { 
        if(!is_online(state))
        {
            return Switch_3_position::invalid;
        }
        return get_switch_3_position(static_cast<uint8_t>(
            (state.key>>2U)&0x03U
        ));
    }

    inline Switch_3_position get_SwR_position(const State &state) { 
        if(!is_online(state))
        {
            return Switch_3_position::invalid;
        }
        return get_switch_3_position(static_cast<uint8_t>(
            state.key&0x03U
        ));
    }


    class Remote_receiver{
    public:
        using State_callback = void(*)(const State &state, void *user);

        explicit Remote_receiver(State_callback callback = nullptr,
             void *user = nullptr, 
             uint32_t timeout_ms = koffline_timeout_ms);
        

        //输入任意长度的串口数据块
        void feed(const uint8_t *data, size_t len,uint32_t arrive_ms);
        //周期性检查掉线
        void poll (uint32_t now_ms);
        
        //串口错误处理和队列丢字节后，清除半帧并进入离线状态
        void break_stream();

        //给接收器的任务使用
        const State &get_state() const { return state_; }

        static uint8_t crc8(
            const uint8_t *data, size_t len
        );
    private:
        static constexpr std::size_t kFrameSize =16U;

        uint8_t frame_[kFrameSize]{};
        std::size_t used_{};

        State state_;

        State_callback callback_;
        void *callback_user_{};
        uint32_t timeout_ms_{};

        //将两个字节转为uint16_t
        static uint16_t read_uint16(const uint8_t* data);
        //删除缓存里面第一个字节
        void drop_front();
        //让缓存开头对齐到 AA 55
        void align_header();
        //校验并解析一条完整帧
        bool decode_frame(uint32_t arrive_ms);
        //把状态交给外部处理
        void notify_callback();
        //将接收器置为离线
        void set_offline();
        

    };
                            
}

