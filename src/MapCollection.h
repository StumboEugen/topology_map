//
// Created by stumbo on 18-5-15.
//

#ifndef TOPOLOGY_MAP_MAPCOLLECTION_H
#define TOPOLOGY_MAP_MAPCOLLECTION_H

#include <set>

#include "MapCandidate.h"

using namespace std;

class MapCollection {
private:
    set<MapCandidate> maps;
};


#endif //TOPOLOGY_MAP_MAPCOLLECTION_H
