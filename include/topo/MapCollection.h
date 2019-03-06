//
// Created by stumbo on 18-5-15.
//

#ifndef TOPOLOGY_MAP_MAPCOLLECTION_H
#define TOPOLOGY_MAP_MAPCOLLECTION_H

#include <list>
#include <iostream>

#include "TopoTools.h"

/**
 * collection of the MapCandidates
 * a member of the mapArranger
 */
class MapCollection {
public:
    void arriveNodeInstance(NodeInstance *, uint8_t arriveAt, double dis_x, double dis_y,
                                double yaw);
    std::list<MapCandidate *>::iterator addNewMap
            (std::list<MapCandidate *>::iterator pos2Insert, MapCandidate *);
    void everyMapThroughGate(gateId exit);

    size_t mapNumbers() {
        return maps.size();
    }

    const std::list<MapCandidate *> & getMaps() const {
        return maps;
    }

    void addNodeDirectly(TopoNode *);

    void addEdgeDirectly(TopoEdge *);

    JSobj toJS() const;

    void clear();

    mapPosInList addMapAtListBack(MapCandidate *);
private:
    /**
     * using list to store the map candidate to sort them quickly
     */
    std::list<MapCandidate*> maps;
};


#endif //TOPOLOGY_MAP_MAPCOLLECTION_H
