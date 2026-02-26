
#include "Tile.h"
#include "arrCustom.h"
#include "Stack.h"
#include "coord.h"
#include "Encoder.h"

class maze{
    public:
        void run_algs();
        maze();
        // void getDetectionJetson();
        // Jetson jetson;
    private:
        void followPath(Stack& path, arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap);
        void dijkstra(coord& start, coord& end, arrCustom<coord>& tilesMap, arrCustom<Tile>& tiles);
        void dfs(arrCustom<coord>& visitedMap, arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap);  
};