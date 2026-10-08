/**
 * @file topic.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>
#include <cstring>
#include "lockfree_queue.hpp"

/*静态内存配置*/

//最大消息结构体字节数
#ifndef TOPICS_MAX_MESSAGE_SIZE
#define TOPICS_MAX_MESSAGE_SIZE 32U
#endif

//最大历史长度（订阅者缓冲队列最大长度）
#ifndef TOPICS_MAX_HISTORY_LEN
#define TOPICS_MAX_HISTORY_LEN 8U
#endif

//最大topic数量
#ifndef TOPICS_MAX_TOPICS
#define TOPICS_MAX_TOPICS 8U
#endif

//某个topics上能搭载的最大订阅
#ifndef TOPICS_MAX_SUBS_PER_TOPIC
#define TOPICS_MAX_SUBS_PER_TOPIC 4U
#endif

struct publish_slot{
    uint8_t data[TOPICS_MAX_MESSAGE_SIZE];
    int len;
};
struct topic_queue_t{
    Algorithm::SpscOverwriteRing<publish_slot, TOPICS_MAX_HISTORY_LEN + 1U> ring;
};

struct internal_topic;
struct subscriber_state;

class TopicPublisher{
public:
    explicit TopicPublisher(const char *topic);
    bool Publish(uint8_t *value, int len) const;

private:
    internal_topic *topic_{nullptr};
    
};

class TopicSubscriber{
public:
    TopicSubscriber(const char *topic, uint32_t buffer_len);
    bool TryGet(publish_slot *out) const ;

private:
    subscriber_state *sub_{nullptr};

};

template<typename T> class TypedTopicPublisher{
public:
    TypedTopicPublisher(const char *topic) : pub_(topic) {}

    bool Publish(const T &value) const{
        return pub_.Publish(
            const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&value)),
            static_cast<int>(sizeof(T)));
    }

private:
    TopicPublisher pub_;
};

template<typename T> class TypedTopicSubscriber{
public:
    explicit TypedTopicSubscriber(const char *topic, uint32_t buffer_len)
        : sub_(topic, buffer_len){}

    bool TryGet(T *out) const {
        if(out == nullptr){
            return false;
        }
        publish_slot packet{};
        if(!sub_.TryGet(&packet)){
            return false;
        }
        if(packet.len != sizeof(T)){
            return false;
        }
        memcpy(out, packet.data, static_cast<int>(sizeof(T)));
        return true;
    }

private:
    TopicSubscriber sub_;

};

void subsQueueInit();