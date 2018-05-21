//
// Created by stumbo on 18-5-15.
//

#ifndef TOPOLOGY_MAP_MAPCOLLECTION_H
#define TOPOLOGY_MAP_MAPCOLLECTION_H

#include <set>

#include "MapCandidate.h"

using namespace std;

class MapCollection {
public:
    void arriveNodeInstance(NodeInstance *, uint8_t arriveAt, double dis_x, double dis_y);
    void addNewMap(MapCandidate *);
private:
    set<MapCandidate*> maps;
};


#endif //TOPOLOGY_MAP_MAPCOLLECTION_H
