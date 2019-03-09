//
// Created by stumbo on 18-5-15.
//

#ifndef TOPOLOGY_MAP_MAPCOLLECTION_H
#define TOPOLOGY_MAP_MAPCOLLECTION_H

#include <set>
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
    void addNewMap(MapCandidate *);
    void everyMapThroughGate(gateId exit);

    size_t mapNumbers() const {
        return maps.size();
    }

    const std::set<MapCandidate *> & getMaps() const {
        return maps;
    }

    MapCandidate * getTheFirstMap() const {
        return maps.begin().operator*();
    }

    void addNodeDirectly(TopoNode *);

    void addEdgeDirectly(TopoEdge *);

    JSobj toJS() const;

    void clear();

private:
    /**
     * using list to store the map candidate to sort them quickly
     */
    std::set<MapCandidate*> maps;
};


#endif //TOPOLOGY_MAP_MAPCOLLECTION_H
