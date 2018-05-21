//
// Created by stumbo on 18-5-21.
//

#ifndef TOPOLOGY_MAP_NODECOLLECTION_H
#define TOPOLOGY_MAP_NODECOLLECTION_H

#include <set>
#include <map>

using namespace std;

class NodeInstance;

class NodeCollection {
public:
    void addInstanceAndCompare(const NodeInstance * instance, uint8_t arriveAt,
                               double dis_x, double dis_y);
private:
    map<int, set<NodeInstance*>> nodeSets;
};


#endif //TOPOLOGY_MAP_NODECOLLECTION_H
