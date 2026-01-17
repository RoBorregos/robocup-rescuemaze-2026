#ifndef ARRCUSTOM_H
#define ARRCUSTOM_H

#include "coord.h"

const int kMaxSize = 300;
const int kMaxInt = 999999;

template<typename T>
class arrCustom {
public:
    arrCustom(int maxSize, T initialValue) : size_(0), maxSize_(maxSize) {
        for(int i = 0; i < maxSize_; i++) {
            data_[i] = initialValue;
        }
    }
    
    T getValue(int index) const {
        if(index >= 0 && index < maxSize_) {
            return data_[index];
        }
        return T();
    }
    
    void set(int index, T value) {
        if(index >= 0 && index < maxSize_) {
            data_[index] = value;
            if(index >= size_) {
                size_ = index + 1;
            }
        }
    }
    
    void push_back(T value) {
        if(size_ < maxSize_) {
            data_[size_] = value;
            size_++;
        }
    }
    
    int getSize() const {
        return size_;
    }
    
    int getIndex(const coord& target) const {
        for(int i = 0; i < size_; i++) {
            if(getValue(i) == target) {
                return i;
            }
        }
        return kMaxInt;
    }
    
    void reset() {
        size_ = 0;
    }
    
    void clear() {
        reset();
    }
    
private:
    T data_[kMaxSize];
    int size_;
    int maxSize_;
};

template<>
inline int arrCustom<coord>::getIndex(const coord& target) const {
    for(int i = 0; i < size_; i++) {
        const coord& current = data_[i];
        if(current.x == target.x && current.y == target.y && current.z == target.z) {
            return i;
        }
    }
    return kMaxInt;
}

#endif