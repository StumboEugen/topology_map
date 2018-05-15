//
// Created by stumbo on 18-5-14.
//

#ifndef TOPOLOGY_MAP_MAPCANDIDATE_H
#define TOPOLOGY_MAP_MAPCANDIDATE_H

#include <memory>
#include <vector>
#include <set>

#include "NodeInstance.h"

using namespace std;

class TopoNode;

class MapCandidate {

private:
    set<TopoNode> nodeIncluded;
};

class TopoNode {

private:
    shared_ptr<NodeInstance> nodeUsed;
    vector<shared_ptr<TopoNode>> exitTo;
};


#endif //TOPOLOGY_MAP_MAPCANDIDATE_H
