//
// Created by stumbo on 18-5-16.
//

#ifndef TOPOLOGY_MAP_TOPONODE_H
#define TOPOLOGY_MAP_TOPONODE_H

#include <iostream>
#include <vector>
#include "TopoTools.h"

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

    /**
     * @return if the gateId corresponded edge is empty
     */
    bool isExitEmpty(gateId number) const {
        return edgeConnected[number] == nullptr;
    }

    void addEdge(gateId number, TopoEdge* edge) {
        edgeConnected[number] = edge;
    }

    TopoEdge *const getEdge(gateId number) const {
        return edgeConnected[number];
    }

    NodeInstance *const getInstanceCorresponding() const {
        return corresponding;
    }

    void disconnectEdge(gateId number) {
        edgeConnected[number] = nullptr;
    }

    //tool member help cloning maps
    TopoNode * clonedTo;

    JSobj toJS() const;

private:
    //the corresponded instance
    NodeInstance *const corresponding;

    //edges connected at this TopoNode
    std::vector<TopoEdge*> edgeConnected;
};

#endif //TOPOLOGY_MAP_TOPONODE_H
