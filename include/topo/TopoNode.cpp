//
// Created by stumbo on 18-5-16.
//
#include "NodeInstance.h"
#include "TopoNode.h"


TopoNode::TopoNode(NodeInstance *const nodeInstance):
        relatedInses(1, nodeInstance),
        edgeConnected(nodeInstance->sizeOfExits(), nullptr),
        tempFlags(0),
        clonedTo(nullptr)
{
    if (!nodeInstance->isAddComplete()) {
        cout << "[TopoNode] bind TopoNode before add complete!" << endl;
    }
}

JSobj TopoNode::toJS() const {
    return JSobj();
}

