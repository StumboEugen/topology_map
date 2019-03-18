//
// Created by stumbo on 18-5-15.
//

#ifndef TOPOLOGY_MAP_MAPCOLLECTION_H
#define TOPOLOGY_MAP_MAPCOLLECTION_H

#include <set>
#include <iostream>

#include "TopoTools.h"

class MapArranger;
/**
 * collection of the MapCandidates
 * a member of the mapArranger
 */
class MapCollection {
public:
    explicit MapCollection(MapArranger * parent);

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

    const std::vector<MapCandidate *> & getOrderedMaps() {
        sortByConfidence(0);
        return orderedMaps;
    }

    MapCandidate * getTheFirstMap() const {
        return maps.begin().operator*();
    }

    void purgeBadMaps(int survival);

    /**
     * sort the map according to confidence, the sorted map will be stored in orderedMaps
     * @param topCount only topCount of maps are stored
     */
    void sortByConfidence(size_t topCount);

    void addNodeDirectly(TopoNode *);

    void addEdgeDirectly(TopoEdge *);

    JSobj toJS() const;

    /**
     * turn the map collection into a JSON obj
     * @param mapCount the count of map you would like to save (0 == whole)
     * @return the JS obj
     */
    JSobj toJSWithSortedMaps(size_t mapCount = 0);

    void clear();

private:
    MapArranger * const parent;
    /**
     * using list to store the map candidate to sort them quickly
     */
    std::set<MapCandidate*> maps;

    /// a temp container
    std::vector<MapCandidate*> orderedMaps;
};


#endif //TOPOLOGY_MAP_MAPCOLLECTION_H
