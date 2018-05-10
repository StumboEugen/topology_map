//
// Created by stumbo on 18-5-10.
//

#ifndef TOPOLOGY_MAP_TOPONODE_H
#define TOPOLOGY_MAP_TOPONODE_H

#include <vector>
#include <set>
#include <string>

#include "NodeExit.h"

using std::vector
using std::set
using std::string

class TopoNode {
public:
    TopoNode();
    int addExit(double dir);
private:
    int exits_count;
    set<NodeExit> exits;
    string extraMsg;
};


#endif //TOPOLOGY_MAP_TOPONODE_H
