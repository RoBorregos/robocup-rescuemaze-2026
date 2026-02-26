#ifndef TILE_DIRECTION_H
#define TILE_DIRECTION_H

#include <cstdint>

enum class TileDirection : uint8_t {
    kNorth = 0,
    kEast = 1,
    kSouth = 2,
    kWest = 3
};

#endif
