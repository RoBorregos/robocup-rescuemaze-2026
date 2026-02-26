#include "maze.h"
#include "Arduino.h"
//#include "../Jetson.h"
//Jetson jetson;
coord inicio = {kBaseCoord, kBaseCoord, kBaseCoord};
coord robotCoord = {kBaseCoord, kBaseCoord, kBaseCoord};
TileDirection directions[4] = {TileDirection::kLeft, TileDirection::kDown, TileDirection::kRight, TileDirection::kUp};
coord checkpointCoord = {kBaseCoord, kBaseCoord, kBaseCoord};
int robotOrientation = 0;
uint8_t level = kBaseCoord;
coord past;
maze::maze(){}
// logic ---------------------------------------------------------
void detection(Tile* curr){
    // Jetson detection disabled
    /*
    if(!curr -> hasVictim()){
        jetson.getDetection();
        if(robot.buttonPressed) return;
        if(robot.victim == 1) robot.harmedVictim();
        else if(robot.victim == 2) robot.stableVictim();
        else if(robot.victim == 3) robot.unharmedVictim();
        if(robot.victim != 0) curr -> setVictim();
        robot.victim = 0;
        robot.kitState=kitID::kNone;
        
    }
    */
}
void maze::followPath(Stack& path, arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap){
    Serial.println("followPath: START");
    Serial.print("followPath: path size = ");
    Serial.println(path.getSize());
    
    const coord& init = path.top();
    Serial.print("followPath: init = ");
    Serial.print(init.x);
    Serial.print(",");
    Serial.println(init.y);
    
    Tile* curr = &tiles.getValue(tilesMap.getIndex(init));
    path.pop();
    
    while(!path.empty()){
        Serial.println("followPath: Loop iteration");
        const coord& next = path.top();
        // curr = &tiles.getValue(tilesMap.getIndex(next));
        Serial.print("followPath: next = ");
        Serial.print(next.x);
        Serial.print(",");
        Serial.println(next.y);
        Serial.println(next.x);
        Serial.println(next.y);
        path.pop(); 
        Serial.println("followPath: Determining rotation...");
        if (next.x > robotCoord.x) {
            Serial.println("followPath: Moving RIGHT (x+1)");
            if(robotOrientation != 90) detection(curr);
            if(robotOrientation == 270) robot.rotate(180);if(robot.buttonPressed) break; detection(curr); robot.rotate(90); if(robot.buttonPressed) break;
            else robot.rotate(90); if(robot.buttonPressed) break; detection(curr);// caso en el que tenga que ir hacia atras en un dead end  
            robotOrientation = 90;
        } else if (next.x < robotCoord.x) {
            if(robotOrientation != 270) detection(curr);
            if(robotOrientation == 90) robot.rotate(180);if(robot.buttonPressed) break; detection(curr); robot.rotate(270); if(robot.buttonPressed) break;
            else robot.rotate(270); if(robot.buttonPressed) break; detection(curr); // caso en el que tenga que ir hacia atras en un dead end  
            robotOrientation = 270;
        } else if (next.y > robotCoord.y) {
            if(robotOrientation != 0) detection(curr); 
            if(robotOrientation == 180)robot.rotate(90);if(robot.buttonPressed) break; detection(curr); robot.rotate(0); if(robot.buttonPressed) break;
            else robot.rotate(0); if(robot.buttonPressed) break; detection(curr); // caso en el que tenga que ir hacia atras en un dead end
            robotOrientation = 0;
        } else if (next.y < robotCoord.y) {
            if(robotOrientation != 180) detection(curr); //caso en el que vaya solo hacia adelante
            if(robotOrientation == 0) robot.rotate(90); if(robot.buttonPressed) break; detection(curr); robot.rotate(180); if(robot.buttonPressed) break; // caso en el que tenga que ir hacia atras en un dead end
            else robot.rotate(180); if(robot.buttonPressed) break; detection(curr); // caso normal en el que tenga que seguir un caminio.
            robotOrientation = 180;
        }
        if(robot.buttonPressed) break;
        Serial.println("followPath: Calling robot.ahead()");
        robot.ahead();
        Serial.println("followPath: robot.ahead() returned");
        if(robot.buttonPressed) break;
        if(robot.blackTile) continue;
        past = robotCoord;
        robotCoord = next;
        curr = &tiles.getValue(tilesMap.getIndex(next));
    }
}
void maze::dijkstra(coord& start, coord& end, arrCustom<coord>& tilesMap, arrCustom<Tile>& tiles){
    Serial.println("dijkstra: START");
    
    // Validate that both start and end are in the map
    if(tilesMap.getIndex(start) == kMaxInt) {
        Serial.println("dijkstra: ERROR - start not in tilesMap!");
        return;
    }
    if(tilesMap.getIndex(end) == kMaxInt) {
        Serial.println("dijkstra: ERROR - end not in tilesMap!");
        return;
    }
    
    // If start == end, no need to move
    if(start == end) {
        Serial.println("dijkstra: start == end, returning");
        return;
    }
    
    Stack path;
    Serial.println("dijkstra: Stack created");
    
    Serial.print("dijkstra: tilesMap.getSize() = ");
    Serial.println(tilesMap.getSize());
    
    Serial.println("dijkstra: Creating explored array...");
    arrCustom<bool> explored(tilesMap.getSize(), false);
    Serial.println("dijkstra: explored created");
    
    Serial.println("dijkstra: Creating distance array...");
    arrCustom<int> distance(tilesMap.getSize(), INT_MAX);
    Serial.println("dijkstra: distance created");
    
    Serial.println("dijkstra: Creating previousPositions array...");
    arrCustom<coord> previousPositions(tilesMap.getSize(), kInvalidPosition);
    Serial.println("dijkstra: previousPositions created");

    Serial.println("dijkstra: Setting distance.set...");
    distance.set(tilesMap.getIndex(start), 0);
    Serial.println("dijkstra: distance.set done");
    
    Serial.println("dijkstra: Setting explored.set...");
    explored.set(tilesMap.getIndex(start), true);
    Serial.println("dijkstra: explored.set done");
    
    uint8_t minDist;
    Serial.println("dijkstra: minDist declared");
    coord current = start;
    Serial.println("dijkstra: current = start");
    //robot.screenPrint("Dijkstra");
    Serial.print("dijkstra: start=(");
    Serial.print(start.x);
    Serial.print(",");
    Serial.print(start.y);
    Serial.print(") end=(");
    Serial.print(end.x);
    Serial.print(",");
    Serial.print(end.y);
    Serial.println(")");
    Serial.println("dijkstra: Entering while loop");
    
    int loopCount = 0;
    const int MAX_LOOPS = 255; // Safety limit
    
    while(!explored.getValue(tilesMap.getIndex(end))){ 
        loopCount++;
        if(loopCount > MAX_LOOPS) {
            Serial.println("dijkstra: ERROR - exceeded MAX_LOOPS, no path found!");
            return;
        }
        
        if(loopCount % 10 == 0) {
            Serial.print("dijkstra: Loop ");
            Serial.print(loopCount);
            Serial.print(" current=(");
            Serial.print(current.x);
            Serial.print(",");
            Serial.print(current.y);
            Serial.println(")");
        }
        
        for(const TileDirection& direction : directions){
            const Tile& currentTile = tiles.getValue(tilesMap.getIndex(current));
            // Check nullptr BEFORE accessing position_
            if(currentTile.adjacentTiles_[static_cast<int>(direction)] != nullptr && !currentTile.hasWall(direction) && !currentTile.hasBlackTile()){
                const coord& adjacent = currentTile.adjacentTiles_[static_cast<int>(direction)] -> position_;
                
                // Validate that adjacent is in tilesMap before accessing distance
                int adjacentIndex = tilesMap.getIndex(adjacent);
                if(adjacentIndex == kMaxInt) {
                    continue; // Skip this adjacent tile
                }
                
                const int weight = currentTile.weights_[static_cast<int>(direction)] + distance.getValue(tilesMap.getIndex(current));
                int currentDist = distance.getValue(adjacentIndex);
                if(weight < currentDist){
                    distance.set(adjacentIndex, weight);
                    previousPositions.set(adjacentIndex, current);
                }
            }
            if(robot.buttonPressed) break;
        }
        
        minDist = kMaxInt;
        //find the minimum distance to the path line
        for(int i = 0; i < tilesMap.getUsed(); i++){
            const coord& currentCoord = tilesMap.getValue(i);
            const int currentDistance = distance.getValue(tilesMap.getIndex(currentCoord));           
            if(currentDistance < minDist && !explored.getValue(tilesMap.getIndex(currentCoord))){
                minDist = currentDistance;
                current = currentCoord;
            }
            if(robot.buttonPressed) break;
        }
        
        // Check if we found a reachable unexplored tile
        if(minDist == kMaxInt) {
            Serial.println("dijkstra: ERROR - No path to end! minDist=kMaxInt");
            Serial.print("dijkstra: Could not reach end=(");
            Serial.print(end.x);
            Serial.print(",");
            Serial.print(end.y);
            Serial.println(")");
            return;
        }
        
        explored.set(tilesMap.getIndex(current),true);
        if(robot.buttonPressed) break;
    }
    current = end;
    while(current != start){
        if(robot.buttonPressed) break;
        //robot.screenPrint("minDist2: ");
        path.push(current);
        current = previousPositions.getValue(tilesMap.getIndex(current));
    }
    path.push(start); //para acceder la tile de inicio en el path. 
    // robot.screenPrint("Path found");
    followPath(path, tiles, tilesMap);
}

void maze::dfs(arrCustom<coord>& visitedMap, arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap){
    Serial.println("dfs: START");
    Stack unvisited;
    unvisited.push(robotCoord);
    coord next;
    Tile* currentTile;
    TileDirection oppositeDirection;
    bool visitedFlag = false;
    bool wall = false;

    while(!unvisited.empty()){
        Serial.println("dfs: Loop iteration");
        coord current = unvisited.top();
        unvisited.pop();
        visitedFlag = false;
        for(int i = 0; i < visitedMap.getSize(); ++i){
            if(visitedMap.getValue(i) == current){
                visitedFlag = true;
                break;
            }
        }
        if (visitedFlag) continue;
        dijkstra(robotCoord, current, tilesMap, tiles);
        visitedMap.push_back(current);
        if(robot.blackTile){
            currentTile = &tiles.getValue(tilesMap.getIndex(current));
            currentTile -> setBlackTile();
            robot.blackTile = false;
            continue;
        }
        if(robot.checkpoint == true){
            currentTile = &tiles.getValue(tilesMap.getIndex(current));
            currentTile -> setCheckpoint();
            checkpointCoord = current;
            robot.checkpoint = false;
        }
        robotCoord = current;
        Serial.print("dfs: robotCoord now at (");
        Serial.print(robotCoord.x);
        Serial.print(",");
        Serial.print(robotCoord.y);
        Serial.println(")");
        //button checkpoint logic
        if(robot.buttonPressed){
            delay(50);
            if(digitalRead(Pins::checkpointPin)==1){


            robot.screenPrint("LoP");      
            robotCoord = inicio;
            robotOrientation = 0;
            robot.screenPrint("Inicio");
            bool braker=false;
            while(true){
                Serial.println("wat");
                robot.screenPrint("Esperando");
                if(!robot.buttonPressed){
                    unsigned long time=millis();
                    while(digitalRead(Pins::checkpointPin)==1){
                        if((millis()-time)>500){
                            ESP.restart();  
                            robot.screenPrint("r");
                            delay(500);
                            // delay(2000);                     
                            // robot.resetOrientation();
                            robot.resetVlx();
                            robot.bno.setupBNO();
                            braker=true;
                            break;
                        }
                    }break;
                }
            }
            robot.screenPrint("Dale");
            Serial.println("resetting visitedMap");
            visitedMap.reset();
            Serial.println("resetting tilesMap");
            tilesMap.reset();
            Serial.println("resetting tiles");
            tiles.reset();
            current = robotCoord;
            unvisited.~Stack();
            unvisited.push(robotCoord);
            tilesMap.push_back(robotCoord);
            tiles.push_back(Tile(robotCoord));
            Serial.println("good");
            if(!braker) robot.checkpointElection();
            robot.resetOrientation();
            robot.buttonPressed=false;
            continue;
        }else{
            robot.buttonPressed=false;
        }
        }
        currentTile = &tiles.getValue(tilesMap.getIndex(current));
        if(robot.rampState != 0){
            int rampDirection = robot.rampState == 1 ? 1 : -1;
            robot.rampState = 0;

            Tile* currTile = &tiles.getValue(tilesMap.getIndex(robotCoord));

            for(int i = 0;i < kNumberOfDirections; i++){
                if(currTile -> adjacentTiles_[i] != nullptr && currTile -> adjacentTiles_[i] -> position_ == past){
                    Tile* pastTile = currTile -> adjacentTiles_[i];

                    current.z += rampDirection;
                    visitedMap.push_back(current);
                    tilesMap.push_back(current);
                    tiles.push_back(Tile(current));
                    Tile* newTile = &tiles.getValue(tilesMap.getIndex(current));

                    for(TileDirection dir: directions){
                        TileDirection opposite; 
                        if(dir == TileDirection::kUp) opposite = TileDirection::kDown;
                        else if(dir == TileDirection::kDown) opposite = TileDirection::kUp;
                        else if(dir == TileDirection::kRight) opposite = TileDirection::kLeft;
                        else if(dir == TileDirection::kLeft) opposite = TileDirection::kRight;
                        if(pastTile -> adjacentTiles_[static_cast<int>(dir)] -> position_ == robotCoord){
                            pastTile -> addAdjacentTile(dir, newTile, false, false);
                            newTile -> addAdjacentTile(opposite, pastTile, false, false);
                            break;
                        }
                    }
                    break;
                }
            }
            
            robotCoord = current;
            
            currentTile = &tiles.getValue(tilesMap.getIndex(current));
            for(const TileDirection direction: directions){
                wall = false; 
                if(robot.isWall(static_cast<int>(direction))) wall = true;
                switch(direction) {
                    case TileDirection::kRight:
                        next = coord{static_cast<uint8_t>(current.x + 1), current.y, static_cast<uint8_t>(current.z)};
                        if(robotOrientation == 270) continue;
                        oppositeDirection = TileDirection::kLeft;
                        break;
                    case TileDirection::kUp:
                        next = coord{current.x, static_cast<uint8_t>(current.y + 1), static_cast<uint8_t>(current.z)};
                        if(robotOrientation == 180) continue;
                        oppositeDirection = TileDirection::kDown;
                        break;
                    case TileDirection::kLeft:
                        next = coord{static_cast<uint8_t>(current.x - 1), current.y, static_cast<uint8_t>(current.z)};
                        if(robotOrientation == 90) continue;
                        oppositeDirection = TileDirection::kRight;
                        break;
                    case TileDirection::kDown:
                        next = coord{current.x, static_cast<uint8_t>(current.y - 1), static_cast<uint8_t>(current.z)};
                        if(robotOrientation == 0) continue;
                        oppositeDirection = TileDirection::kUp;
                        break;
                }
                if(currentTile -> adjacentTiles_[static_cast<int>(direction)] == nullptr){
                    int index = tilesMap.getIndex(next);
                    Tile* nextTile;
                    if (index == kMaxInt) { 
                        tilesMap.push_back(next);
                        tiles.push_back(Tile(next));
                        // IMPORTANT: Reassign currentTile after push_back as array may have been reallocated
                        currentTile = &tiles.getValue(tilesMap.getIndex(current));
                        nextTile = &tiles.getValue(tilesMap.getIndex(next));
                    } else {  
                        nextTile = &tiles.getValue(index);
                    }
                
                    if (nextTile -> position_ == kInvalidPosition) nextTile -> setPosition(next);
                    // join the tiles and if there is no wall between them
                    currentTile -> addAdjacentTile(direction, nextTile, wall,false);
                    nextTile -> addAdjacentTile(oppositeDirection, currentTile, wall, false);
                    if(!wall){
                        visitedFlag = false;
                        for(uint8_t i = 0; i < visitedMap.getSize(); ++i){
                            if(visitedMap.getValue(i) == next){
                                visitedFlag = true;
                                break;
                            }
                        }
                        if(!visitedFlag){ 
                            unvisited.push(next);
                            //robot.screenPrint("Entre");
                        }
                    }
                    
                }
            }
            continue;
        }else{
        for(const TileDirection direction: directions){
            wall = false; 
            if(robot.isWall(static_cast<int>(direction))) wall = true;
            switch(direction) {
                case TileDirection::kRight:
                    next = coord{static_cast<uint8_t>(current.x + 1), current.y, static_cast<uint8_t>(current.z)};
                    oppositeDirection = TileDirection::kLeft;
                    break;
                case TileDirection::kUp:
                    next = coord{current.x, static_cast<uint8_t>(current.y + 1), static_cast<uint8_t>(current.z)};
                    oppositeDirection = TileDirection::kDown;
                    break;
                case TileDirection::kLeft:
                    next = coord{static_cast<uint8_t>(current.x - 1), current.y, static_cast<uint8_t>(current.z)};
                    oppositeDirection = TileDirection::kRight;
                    break;
                case TileDirection::kDown:
                    next = coord{current.x, static_cast<uint8_t>(current.y - 1),static_cast<uint8_t>(current.z)};
                    oppositeDirection = TileDirection::kUp;
                    break;
            }
            //check for adjacentTiles and connecting them
            if(currentTile -> adjacentTiles_[static_cast<int>(direction)] == nullptr){
                int index = tilesMap.getIndex(next);
                Tile* nextTile;
                if (index == kMaxInt) { 
                    tilesMap.push_back(next);
                    tiles.push_back(Tile(next));
                    // IMPORTANT: Reassign currentTile after push_back as array may have been reallocated
                    currentTile = &tiles.getValue(tilesMap.getIndex(current));
                    nextTile = &tiles.getValue(tilesMap.getIndex(next));
                } else {  
                    nextTile = &tiles.getValue(index);
                }
            
                if (nextTile -> position_ == kInvalidPosition) nextTile -> setPosition(next);
                // join the tiles and if there is no wall between them
                currentTile -> addAdjacentTile(direction, nextTile, wall,false);
                if(robot.blueTile){
                    nextTile -> addAdjacentTile(oppositeDirection, nextTile, wall, true);
                    robot.blueTile = false;
                }else{
                    nextTile -> addAdjacentTile(oppositeDirection, currentTile, wall, false);
                }
                if(!wall){
                    visitedFlag = false;
                    for(uint8_t i = 0; i < visitedMap.getSize(); ++i){
                        if(visitedMap.getValue(i) == next){
                            visitedFlag = true;
                            break;
                        }
                    }
                    if(!visitedFlag){  
                        unvisited.push(next);
                    }
                }
                
            }
        }
        }
    }

    if(unvisited.empty()){ /*robot.screenPrint("Unvisited empty");*/ delay(5000);}
    if(robot.buttonPressed==false) dijkstra(robotCoord, inicio, tilesMap, tiles); 
}
void maze::run_algs(){
    Serial.println("run_algs: START");
    
    // while(!robot.buttonPressed) robot.screenPrint("W Inicio"); 
    // robot.buttonPressed = !robot.buttonPressed;
    // robot.resetOrientation();
    
    Serial.println("run_algs: Creating visitedMap...");
    arrCustom<coord> visitedMap(kMaxSize, kInvalidPosition);
    
    Serial.println("run_algs: Creating tilesMap...");
    arrCustom<coord> tilesMap(kMaxSize, kInvalidPosition);
    
    Serial.println("run_algs: Creating tiles...");
    arrCustom<Tile> tiles(kMaxSize, Tile(kInvalidPosition));
    
    Serial.println("run_algs: push_back robotCoord to tilesMap...");
    tilesMap.push_back(robotCoord);
    
    Serial.print("run_algs: Getting index of robotCoord: ");
    uint8_t idx = tilesMap.getIndex(robotCoord);
    Serial.println(idx);
    
    Serial.println("run_algs: Creating Tile and assigning...");
    tiles.getValue(idx) = Tile(robotCoord);
    
    Serial.println("run_algs: Calling dfs...");
    dfs(visitedMap, tiles, tilesMap);
    
    Serial.println("run_algs: END");
}