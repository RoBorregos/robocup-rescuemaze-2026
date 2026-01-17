#ifndef HEAP_H
#define HEAP_H

#include "coord.h"

struct HeapNode {
    coord position;
    int distance;
    
    HeapNode() : position(kInvalidPosition), distance(0) {}
    HeapNode(coord pos, int dist) : position(pos), distance(dist) {}
};

const int kMaxHeapSize = 200;

class MinHeap {
public:
    MinHeap() : size_(0) {}
    
    bool empty() const { return size_ == 0; }
    int size() const { return size_; }
    
    void push(coord position, int distance) {
        if(size_ >= kMaxHeapSize) return;
        data_[size_].position = position;
        data_[size_].distance = distance;
        size_++;
        bubbleUp(size_ - 1);
    }
    
    HeapNode top() const {
        if(size_ > 0) return data_[0];
        return HeapNode();
    }
    
    void pop() {
        if(size_ == 0) return;
        data_[0] = data_[size_ - 1];
        size_--;
        if(size_ > 0) bubbleDown(0);
    }
    
    void clear() { size_ = 0; }
    
private:
    HeapNode data_[kMaxHeapSize];
    int size_;
    
    int parent(int i) { return (i - 1) / 2; }
    int leftChild(int i) { return 2 * i + 1; }
    int rightChild(int i) { return 2 * i + 2; }
    
    void swap(int i, int j) {
        HeapNode temp = data_[i];
        data_[i] = data_[j];
        data_[j] = temp;
    }
    
    void bubbleUp(int index) {
        while(index > 0) {
            int p = parent(index);
            if(data_[index].distance < data_[p].distance) {
                swap(index, p);
                index = p;
            } else break;
        }
    }
    
    void bubbleDown(int index) {
        while(true) {
            int smallest = index;
            int left = leftChild(index);
            int right = rightChild(index);
            
            if(left < size_ && data_[left].distance < data_[smallest].distance) {
                smallest = left;
            }
            if(right < size_ && data_[right].distance < data_[smallest].distance) {
                smallest = right;
            }
            if(smallest != index) {
                swap(index, smallest);
                index = smallest;
            } else break;
        }
    }
};

#endif