#ifndef STACK_H
#define STACK_H

#include "coord.h"

const int kMaxStackSize = 200;

class Stack {
public:
    Stack() : size_(0) {}
    ~Stack() {}
    
    void push(const coord& value) {
        if(size_ < kMaxStackSize) {
            data_[size_] = value;
            size_++;
        }
    }
    
    coord pop() {
        if(size_ > 0) {
            size_--;
            return data_[size_];
        }
        return kInvalidPosition;
    }
    
    const coord& top() const {
        if(size_ > 0) {
            return data_[size_ - 1];
        }
        static coord invalid = kInvalidPosition;
        return invalid;
    }
    
    bool empty() const {
        return size_ == 0;
    }
    
    int size() const {
        return size_;
    }
    
    void clear() {
        size_ = 0;
    }
    
private:
    coord data_[kMaxStackSize];
    int size_;
};

#endif