//
// Created by stumbo on 18-5-16.
//
#include "NodeInstance.h"
#include "TopoNode.h"
#include "TopoEdge.h"

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

void TopoNode::ringRotate(int diff) {
    auto size = edgeConnected.size();
    if (diff < 0) {
        diff += size * (-diff / size + 1);
    } else {
        diff %= size;
    }

    auto copy = edgeConnected;

    for (int i = 0; i < size - diff; ++i) {
        edgeConnected[i + diff] = copy[i];
        if (copy[i])
            copy[i]->resetGate(this, static_cast<gateId>(i + diff));
    }

    for (auto i = size - diff; i < size; ++i) {
        edgeConnected[i + diff - size] = copy[i];
        if (copy[i])
            copy[i]->resetGate(this, static_cast<gateId>(i + diff - size));
    }
}

