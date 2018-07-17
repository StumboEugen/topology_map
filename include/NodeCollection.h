//
// Created by stumbo on 18-5-21.
//

#ifndef TOPOLOGY_MAP_NODECOLLECTION_H
#define TOPOLOGY_MAP_NODECOLLECTION_H

#include <set>
#include <map>

using namespace std;

class NodeInstance;
class MapCandidate;

/**
 * collection of the NodeInstances
 * a member in MapArranger
 */
class NodeCollection {
public:
    vector<MapCandidate *> addInstanceAndCompare(NodeInstance * instance, uint8_t arriveAt,
                                                 double dis_x, double dis_y);
private:
    map<int, set<NodeInstance*>> nodeSets;
};
//TODO 更多的分类标准


#endif //TOPOLOGY_MAP_NODECOLLECTION_H
