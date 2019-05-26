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
    JSobj obj;
    for (const auto & ins: relatedInses) {
        obj.append(ins->getSerialNumber());
    }
    return std::move(obj);
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

