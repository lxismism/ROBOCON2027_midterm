/**
 * @file lockfree_queue.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include <atomic>
#include <type_traits>
#include <utility>

namespace Algorithm{

enum class QueueError : uint8_t { OK = 0, FULL, EMPTY};

template <typename Data, size_t Capacity> class alignas(32) MpscQueue {
    
    static_assert(Capacity >= 2, "Capacity must be >= 2");
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be power of two");
    static_assert(std::is_move_assignable<Data>::value ||
                  std::is_copy_assignable<Data>::value, "Capacity must be move_assignable or copy_assignable");

public:                  
    MpscQueue() noexcept : enqueue_pos_(0), dequeue_pos_(0) {
        for(size_t i = 0; i < Capacity; i++){
            slots_[i].sequence.store(i, std::memory_order_relaxed);
        }
    }
    MpscQueue(const MpscQueue &) =delete;
    MpscQueue &operator=(const MpscQueue &) =delete;

    template<typename T> QueueError TryPush(T &&item) noexcept {
        size_t pos = enqueue_pos_.load(std::memory_order_relaxed); 

        for(;;){
            Slot &slot = slots_[pos & kMask];
            const size_t seq = slot.sequence.load(std::memory_order_acquire);
            const  intptr_t diff = static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos);
            
            if(diff == 0){
                if(enqueue_pos_.compare_exchange_weak(pos,pos + 1, 
                                    std::memory_order_relaxed,
                                    std::memory_order_relaxed)){     /*weak跟strong有什么区别呢？*/
                
                    slot.data = std::forward<T>(item);
                    slot.sequence.store(pos+1 , std::memory_order_release);
                    return QueueError::OK;
                }
            }
            else if(diff < 0){
                return QueueError::FULL;
            }
            else pos = enqueue_pos_.load(std::memory_order_relaxed);

        }

    }
    QueueError TryPop(Data &out) noexcept {
        const size_t pos = dequeue_pos_.load(std::memory_order_relaxed);
        Slot &slot = slots_[pos & kMask];
        const size_t seq = slot.sequence.load(std::memory_order_acquire);
        const intptr_t diff = static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos + 1);

        if(diff == 0){
            out = std::move(slot.data);
            slot.sequence.store(pos + Capacity, std::memory_order_release);
            dequeue_pos_.store(pos + 1, std::memory_order_relaxed);
            return QueueError::OK;
        }
        else if(diff < 0){
            return QueueError::EMPTY;
        }
        else{
            return QueueError::EMPTY;
        }
        
    }

    size_t Size() const noexcept {
        const size_t enq = enqueue_pos_.load(std::memory_order_acquire);
        const size_t deq = dequeue_pos_.load(std::memory_order_acquire);
        return enq - deq;
    }
    
    static constexpr size_t GetCapacity() noexcept { return Capacity; }



private:
    struct alignas(32) Slot{
        Data data;
        std::atomic<size_t> sequence;
    };

    alignas(32) Slot slots_[Capacity];

    static constexpr size_t kMask = Capacity - 1;
    alignas(32) std::atomic<size_t> enqueue_pos_;
    alignas(32) std::atomic<size_t> dequeue_pos_;

};

template <typename T, uint32_t MaxCapacity> class SpscOverwriteRing{
public:
    SpscOverwriteRing() = default;

    void Init(uint32_t capacity){
        if(capacity == 0U){
            capacity = 1U;
        }else if(capacity > MaxCapacity){
            capacity = MaxCapacity;
        }
        capacity_ = capacity;   
        head_.store(0U,std::memory_order_relaxed);
        tail_.store(0U,std::memory_order_relaxed);
    }

    bool IsReady() const { return capacity_ > 0U ;}

    void Clear(){
        if(!IsReady()){
            return;
        }
        head_.store(0U,std::memory_order_relaxed);
        tail_.store(0U,std::memory_order_relaxed);
    }


    void Push(const T &item){
        if(!IsReady()){ 
            return; 
        }


        uint32_t tail = tail_.load(std::memory_order_relaxed);
        uint32_t next_tail = Inc(tail);
        uint32_t head = head_.load(std::memory_order_acquire);

        if(next_tail == head){
            head_.store(Inc(head),std::memory_order_release);
        }

        items_[tail] = item;
        tail_.store(next_tail, std::memory_order_release);
    }

    bool Pop(T *out){
        if(out == nullptr || capacity_ == 0U){
            return false;
        }

        uint32_t head = head_.load(std::memory_order_relaxed);
        uint32_t tail = tail_.load(std::memory_order_acquire);
        
        if(head == tail) return false;

        *out = items_[head];
        head_.store(Inc(head),std::memory_order_release);

        return true;
    }



private:
    T items_[MaxCapacity]{};
    uint32_t capacity_{0};     
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};

    uint32_t Inc(uint32_t index){ return (index + 1U) % capacity_; }
};

}
