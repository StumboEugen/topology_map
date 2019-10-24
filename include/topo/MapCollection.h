//
// Created by stumbo on 18-5-15.
//

#ifndef TOPOLOGY_MAP_MAPCOLLECTION_H
#define TOPOLOGY_MAP_MAPCOLLECTION_H

#include <set>
#include <iostream>

#include "TopoTools.h"
#include "TopoPath.h"

class MapArranger;
/**
 * container of MapCandidate s.
 * a member of the MapArranger.
 */
class MapCollection {
public:
    // constructor of MapCollection.
    explicit MapCollection(MapArranger * parent);

    // deduce every MapCandidate in this to arrive at next NodeInstance.
    void arriveNodeInstance(NodeInstance *, uint8_t arriveAt, double dis_x, double dis_y,
                                double yaw);

    // directly add a new MapCandidate
    void addNewMap(MapCandidate *);

    // set every MapCandidate 's robot to move throuth a gate.
    void everyMapThroughGate(gateId exit);

    // purge assigned number of maps according to the confidence.
    void purgeBadMaps(int survivorCount);

    // sort the map according to confidence, the sorted map will be stored in orderedMaps
    void sortByConfidence(size_t topCount);

    // add TopoNode to MapCollection directly.
    void addNodeDirectly(TopoNode *);

    // add TopoEdge to MapCollection directly.
    void addEdgeDirectly(TopoEdge *);

    // convert MapCollection to JSON structure
    JSobj toJS() const;

    // convert MapCollection to JSON structure with a limited amount of maps
    JSobj toJSWithSortedMaps(size_t mapCount = 0);

    // clear and destroy all MapCandidates.
    void clear();

    // calculate the sum of confidence
    double calSumOfConfidence();

public:
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

    double getSumOfConfidence() const {
        return sumOfConfidence;
    }

private:
    /// the MapArranger that owns this MapCollection
    MapArranger * const parent;

    /// for quick remove, the maps are stored in std::set
    std::set<MapCandidate*> maps;

    /// a temp container for ordered maps. usually it contians all maps in set and the first
    /// TopCount of maps are in order
    /// @see sortByConfidence()
    std::vector<MapCandidate*> orderedMaps;

    /// the sum of all maps' confidence, for easy normalization
    /// @see calSumOfConfidence()
    double sumOfConfidence = 0.0;

    TopoPath currentPath;
};


#endif //TOPOLOGY_MAP_MAPCOLLECTION_H
