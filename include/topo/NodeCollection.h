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

/**
 * collection of the NodeInstances
 * a member in MapArranger
 */
class NodeCollection {
public:
    vector<pair<mapPosInList, MapCandidate *>>
    addInstanceAndCompare(NodeInstance *instance, uint8_t arriveAt,
                          double dis_x, double dis_y);

    const map<int, set<NodeInstance *>> &getNodeSets() const {
        return nodeSets;
    }

    size_t size() const {
        return nodeSets.size();
    }

    JSobj toJS() const;

    void clear();

    void addInstanceDirectly(NodeInstance * newNode);

private:
    map<int, set<NodeInstance*>> nodeSets;
};
//TODO 更多的分类标准


#endif //TOPOLOGY_MAP_NODECOLLECTION_H
