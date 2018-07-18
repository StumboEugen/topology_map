//
// Created by stumbo on 18-5-15.
//

#ifndef TOPOLOGY_MAP_MAPCOLLECTION_H
#define TOPOLOGY_MAP_MAPCOLLECTION_H

#include <set>

#include "TopoType.h"

using namespace std;

class NodeInstance;
class MapCandidate;

/**
 * collection of the MapCandidates
 * a member of the mapArranger
 */
class MapCollection {
public:
    void arriveNodeInstance(NodeInstance *, uint8_t arriveAt, double dis_x, double dis_y);
    void addNewMap(MapCandidate *);
    void everyMapThroughGate(gateId exit);

    size_t mapNumbers() {
        return maps.size();
    }
private:
    set<MapCandidate*> maps;
};


#endif //TOPOLOGY_MAP_MAPCOLLECTION_H
