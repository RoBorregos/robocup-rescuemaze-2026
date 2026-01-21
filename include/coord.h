#ifndef COORD_H
#define COORD_H

#include <stdint.h>

struct coord {
    uint8_t x;
    uint8_t y;
    uint8_t z;
    
    bool operator==(const coord& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    bool operator!=(const coord& other) const {
        return !(*this == other);
    }
};

const coord kInvalidPosition = {255, 255, 255};
const uint8_t kBaseCoord = 100;
const uint8_t kMaxSize = 255;
const int kNumberOfDirections = 4;

#endif
