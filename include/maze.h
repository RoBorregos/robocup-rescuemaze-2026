
#ifndef MAZE_H
#define MAZE_H

// Include all data structures
#include "coord.h"
#include "Tile.h"
#include "arrCustom.h"
#include "stack.h"
#include "heap.h"
#include "encoder.h"
#include "robot.h"


int calculateTargetOrientation(const coord& current, const coord& next);

// Rotates robot to target orientation using shortest rotation path
void rotateToOrientation(int targetOrientation, Tile* curr);

TileDirection getOppositeDirection(TileDirection dir);

// Calculates the next coordinate based on current position and direction
coord getNextCoord(const coord& current, TileDirection direction);

bool isVisited(const coord& pos, arrCustom<coord>& visitedMap);


void detection(Tile* curr);

class maze {
public:
    maze();
    void run_algs();

private:

    void dfs(arrCustom<coord>& visitedMap, arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap);
    
    void dijkstra(coord& start, coord& end, arrCustom<coord>& tilesMap, arrCustom<Tile>& tiles);
    
   
    void followPath(Stack& path, arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap);
//helpers
   
    void handleRamp(coord& current, arrCustom<coord>& visitedMap, 
                    arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap);
    
   
    void exploreNeighbors(coord& current, Tile* currentTile, Stack& unvisited,
                         arrCustom<coord>& visitedMap, arrCustom<Tile>& tiles, 
                         arrCustom<coord>& tilesMap, bool afterRamp);
};

#endif // MAZE_H