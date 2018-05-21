//
// Created by stumbo on 18-5-16.
//
#include "NodeInstance.h"
#include "TopoNode.h"


TopoNode::TopoNode(const NodeInstance *const nodeInstance):
        corresponding(nodeInstance),
        edgeConnected(nodeInstance->sizeOfExits(), nullptr),
        clonedTo(nullptr)
{
    if (!nodeInstance->isAddComplete()) {
        cout << "bind TopoNode before add complete!" << endl;
    }
}
