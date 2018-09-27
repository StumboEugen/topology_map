//
// Created by stumbo on 18-5-21.
//

#ifndef TOPOLOGY_MAP_NODECOLLECTION_H
#define TOPOLOGY_MAP_NODECOLLECTION_H

#include <set>
#include <map>
#include <unordered_map>

#include "TopoTools.h"

using namespace std;

class NodeInstance;
class MapCandidate;

/**
 * collection of the NodeInstances
 * a member in MapArranger
 */
class NodeCollection {
public:
    vector<pair<list<MapCandidate *, std::allocator<MapCandidate *>>::iterator, MapCandidate *>>
    addInstanceAndCompare(NodeInstance *instance, uint8_t arriveAt,
                          double dis_x, double dis_y);

    const map<int, set<NodeInstance *>> &getNodeSets() const {
        return nodeSets;
    }

    JSobj toJS() const;

    void clear();

private:
    map<int, set<NodeInstance*>> nodeSets;
};
//TODO 更多的分类标准


#endif //TOPOLOGY_MAP_NODECOLLECTION_H
