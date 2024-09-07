#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

template <typename T, size_t N>
class CircularBuffer {
private:
    T buffer_[N];
    uint8_t head_;
    uint8_t tail_;
    size_t size_;

public:
    CircularBuffer() : head_(0), tail_(0), size_(0) {
    }

    bool push(const T& item) {
        if (size_ == N) {
            return false;
        }
        buffer_[head_] = item;
        head_ = (head_ + 1) % N;
        size_++;
        return true;
    }

    bool pop(T& item) {
        if (size_ == 0) {
            return false;
        }
        item = buffer_[tail_];
        tail_ = (tail_ + 1) % N;
        size_--;
        return true;
    }

    size_t size() const {
        return size_;
    }

    bool empty() const {
        return size_ == 0;
    }

    bool full() const {
        return size_ == N;
    }

    bool watch(uint8_t num, T& item) {
        item = buffer_[num % N];
        return true;
    }

    bool trash(uint8_t num) {
        if (size_ < num) {
            return false;
        }
        T item;
        while (num < size_) {
            pop(item);
        }
        return true;
    }

    uint8_t get_head() const {
        return head_;
    }

    void clear() {
        head_ = 0;
        tail_ = 0;
        size_ = 0;
    }
};
