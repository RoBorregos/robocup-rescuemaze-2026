
#ifndef COORD_H
#define COORD_H
#include <stdint.h>

// Base coordinate value (usually 100 to avoid negative numbers)
const uint8_t kBaseCoord = 100;
// Invalid position marker
const coord kInvalidPosition = {255, 255, 255};
struct coord {
    uint8_t x;  // X position in maze
    uint8_t y;  // Y position in maze
    uint8_t z;  // Z level (for ramps)
    
    // Comparison operators for easy coordinate comparison
    bool operator==(const coord& other) const {
        return (x == other.x && y == other.y && z == other.z);
    }
    
    bool operator!=(const coord& other) const {
        return !(*this == other);
    }
};
#endif // COORD_H