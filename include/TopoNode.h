//
// Created by stumbo on 18-5-16.
//

#ifndef TOPOLOGY_MAP_TOPONODE_H
#define TOPOLOGY_MAP_TOPONODE_H

#include <iostream>
#include <vector>

using std::cout;
using std::endl;

class NodeInstance;
class TopoEdge;

/**
 * represent a node in map candidate
 */
class TopoNode {

public:
    explicit TopoNode(NodeInstance * nodeInstance);

    bool isExitEmpty(uint8_t number) const {
        return edgeConnected[number] == nullptr;
    }

    void addEdge(uint8_t number, TopoEdge* edge) {
        edgeConnected[number] = edge;
    }

    TopoEdge *const getEdge(uint8_t number) const {
        return edgeConnected[number];
    }

    NodeInstance *const getInstance() const {
        return corresponding;
    }

    void disconnectEdge(uint8_t number) {
        edgeConnected[number] = nullptr;
    }

    TopoNode * clonedTo;

private:
    NodeInstance *const corresponding;
    std::vector<TopoEdge*> edgeConnected;
};

#endif //TOPOLOGY_MAP_TOPONODE_H
