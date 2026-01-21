#ifndef TILE_H
#define TILE_H

#include "coord.h"
#include <stdint.h>

const int kNumberOfDirections = 4;

enum class TileDirection {
    kUp = 0,
    kDown = 1,
    kLeft = 2,
    kRight = 3
};

class Tile {
public:
    Tile(coord position) : position_(position) {
        for(int i = 0; i < kNumberOfDirections; i++) {
            adjacentTiles_[i] = nullptr;
            walls_[i] = false;
            weights_[i] = 1;
        }
        victim_ = false;
        checkpoint_ = false;
        blackTile_ = false;
    }
    
    Tile() : Tile(kInvalidPosition) {}
    
    bool hasVictim() const { return victim_; }
    bool hasCheckpoint() const { return checkpoint_; }
    bool hasBlackTile() const { return blackTile_; }
    
    bool hasWall(TileDirection direction) const {
        return walls_[static_cast<int>(direction)];
    }
    
    void setVictim() { victim_ = true; }
    void setCheckpoint() { checkpoint_ = true; }
    void setBlackTile() { blackTile_ = true; }
    void setPosition(coord pos) { position_ = pos; }
    
    void addAdjacentTile(TileDirection direction, Tile* tile, bool wall, bool isSpeedBump) {
        int dirIndex = static_cast<int>(direction);
        adjacentTiles_[dirIndex] = tile;
        walls_[dirIndex] = wall;
        if(isSpeedBump) {
            weights_[dirIndex] = 3;
        } else {
            weights_[dirIndex] = 1;
        }
    }
    
    coord position_;
    Tile* adjacentTiles_[kNumberOfDirections];
    int weights_[kNumberOfDirections];
    
private:
    bool walls_[kNumberOfDirections];
    bool victim_;
    bool checkpoint_;
    bool blackTile_;
};

#endif