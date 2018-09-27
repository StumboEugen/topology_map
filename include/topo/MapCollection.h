//
// Created by stumbo on 18-5-15.
//

#ifndef TOPOLOGY_MAP_MAPCOLLECTION_H
#define TOPOLOGY_MAP_MAPCOLLECTION_H

#include <list>

#include "TopoTools.h"

class NodeInstance;
class MapCandidate;

/**
 * collection of the MapCandidates
 * a member of the mapArranger
 */
class MapCollection {
public:
    void arriveNodeInstance(NodeInstance *, uint8_t arriveAt, double dis_x, double dis_y);
    std::list<MapCandidate *>::iterator addNewMap
            (std::list<MapCandidate *>::iterator pos2Insert, MapCandidate *);
    void everyMapThroughGate(gateId exit);

    size_t mapNumbers() {
        return maps.size();
    }

    JSobj toJS() const;
private:
    /**
     * using list to store the map candidate to sort them quickly
     */
    std::list<MapCandidate*> maps;
};


#endif //TOPOLOGY_MAP_MAPCOLLECTION_H
