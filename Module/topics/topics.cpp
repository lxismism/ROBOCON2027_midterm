/**
 * @file topic.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "topics.hpp"
#include "lockfree_queue.hpp"
#include <cstring>

struct internal_topic{
    const char* topic_str;
    subscriber_state *subs;
    uint32_t sub_count;
    struct internal_topic *next;
    bool in_use;
    
};

struct subscriber_state{
    topic_queue_t queue;
    uint32_t buffer_len;
    internal_topic *topic;
    struct subscriber_state *next;
    bool in_use;
};

static internal_topic g_topic_pool[TOPICS_MAX_TOPICS];
static subscriber_state 
    g_sub_pool[TOPICS_MAX_TOPICS * TOPICS_MAX_SUBS_PER_TOPIC];

static internal_topic *AllocateTopic(const char *topic) {
    for(auto &item : g_topic_pool){
        if(!item.in_use){
            item.topic_str = topic;
            item.subs = nullptr;
            item.sub_count = 0U;
            item.next = nullptr;
            item.in_use = true;
            return &item;
        }
    }
    return nullptr;
}

static subscriber_state *AllocateSubscriber(internal_topic *topic, uint32_t buffer_len){
    if(topic == nullptr || topic->topic_str == nullptr ||
       topic->sub_count >= TOPICS_MAX_SUBS_PER_TOPIC ||
       buffer_len <= 0U|| buffer_len > TOPICS_MAX_HISTORY_LEN) return nullptr;
    for(auto &item : g_sub_pool){
        if(!item.in_use){
            item.topic = topic;
            item.next = nullptr;
            item.buffer_len = buffer_len;
            item.in_use = true;
            topic->sub_count += 1U;                                               
            return &item;
        }
    }
    return nullptr;
}


bool TopicPublisher::Publish(uint8_t* data, int len) const {
    /*防御性编程：防topic_空指针，防空data，len必须在区间内*/
    if(topic_ == nullptr || data == nullptr || len <= 0 ||
       len > static_cast<int>(TOPICS_MAX_MESSAGE_SIZE))
    {
        return false;
    }   

    /*定义slot，并深拷贝*/
    publish_slot slot{};
    memcpy(slot.data, data, static_cast<size_t>(len));
    slot.len = len;

    /*这里用了节点，不直接操作topic_->subs了，编程习惯*/
    subscriber_state *node = topic_->subs;
    while(node != nullptr){
        /*把深拷贝的数据slot推进ring，队列里的数据都是有物理内存的*/
        node->queue.ring.Push(slot);
        node = node->next;
    }
    
    return true;
}

bool TopicSubscriber::TryGet(publish_slot *out) const {
    if(sub_ == nullptr || sub_->topic ==nullptr ||
        out == nullptr){
        return false;
    }

    if(!sub_->queue.ring.Pop(out)){
        return false;
    }
    return true;
    
}


class TopicBus{
public:
    static TopicBus &Instance(){
        static TopicBus Instance;
        return Instance;
    }
    internal_topic* GetTopicHead() const { return topics_; }

    internal_topic *RegisterTopic(const char *topic){
        if(topic == nullptr) return nullptr;

        internal_topic *now = nullptr;

        for(now = topics_; now != nullptr; now = now->next){
            if(now->topic_str != nullptr && 
               std::strcmp(now->topic_str, topic) == 0){
                return now;
            }
        }

        now = AllocateTopic(topic);
        if(now == nullptr){
            return nullptr;
        }
        now->next = topics_;
        topics_ = now;
        return now;
    }

private:
    TopicBus() = default;
    internal_topic* topics_{nullptr};

};

TopicPublisher::TopicPublisher(const char *topic)
        : topic_(TopicBus::Instance().RegisterTopic(topic)){}

TopicSubscriber::TopicSubscriber(const char *topic, uint32_t buffer_len){

    internal_topic *now_topic = TopicBus::Instance().RegisterTopic(topic);
    if(now_topic == nullptr) return; 
    subscriber_state *now_sub = AllocateSubscriber(now_topic, buffer_len);
    if(now_sub == nullptr) return;
    
    now_sub->next = now_topic->subs;
    now_topic->subs = now_sub;
    sub_ = now_sub;

}

void subsQueueInit(){
    for(internal_topic *topic_id = TopicBus::Instance().GetTopicHead();
            topic_id != nullptr; topic_id = topic_id->next){
        for(subscriber_state *subs_id = topic_id->subs;
                subs_id != nullptr; subs_id = subs_id->next){
                    subs_id->queue.ring.Init(subs_id->buffer_len + 1U);
                }
    }
}