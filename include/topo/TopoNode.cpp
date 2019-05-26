//
// Created by stumbo on 18-5-16.
//
#include "NodeInstance.h"
#include "TopoNode.h"

/**
 * create a TopoNode according to the nodeInstance
 * @param nodeInstance
 */
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

/**
 * clone a TopoNode, including the relatedInses
 * @param that
 */
TopoNode::TopoNode(const TopoNode *that) :
        relatedInses(that->relatedInses),
        edgeConnected(that->edgeConnected.size(), nullptr),
        tempFlags(0),
        clonedTo(nullptr)
{
}

