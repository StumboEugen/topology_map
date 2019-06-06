//
// Created by stumbo on 18-5-16.
//
#include "NodeInstance.h"
#include "TopoNode.h"
#include "TopoEdge.h"

/// create a TopoNode according to the nodeInstance
/**
 * @note here we didn't access to the NodeInstance 's usages, the work is done by the caller
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

/// convert the TopoNode to JSON structure
/**
 * @return
 * [] array of related NodeInstances
 */
JSobj TopoNode::toJS() const {
    JSobj obj;
    for (const auto & ins: relatedInses) {
        obj.append(ins->getSerialNumber());
    }
    return std::move(obj);
}

/// copy constructor from another TopoNode
/**
 * clone a TopoNode, including the relatedInses, but don;t modify the usage
 * @param that another TopoNode
 * @warning the related NodeInstance's useage is not modified, the work belongs to the caller
 * because we don't know the usage arrangments here
 */
TopoNode::TopoNode(const TopoNode *that) :
        relatedInses(that->relatedInses),
        edgeConnected(that->edgeConnected.size(), nullptr),
        tempFlags(0),
        clonedTo(nullptr)
{
}

/// rotate the exits' ID
/**
 * it is because, sometimes two NodeInstance from a same place (in real world) has different
 * index caused by margin effect at west direction(-pi to pi)
 * @param diff the diff that this TopoNode needs to plus
 *
 * @note \b example \n
 * if diff = 1, then the gate id will be changed like this: 0123 -> 3012
 *
 * @attention we assume that the diff will never be 0, so we don't check if it is 0
 *
 * @see MapCandidate::arriveAtSimiliar()
 */
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
            copy[i]->resetGateNO(this, static_cast<gateId>(i + diff));
    }

    for (auto i = size - diff; i < size; ++i) {
        edgeConnected[i + diff - size] = copy[i];
        if (copy[i])
            copy[i]->resetGateNO(this, static_cast<gateId>(i + diff - size));
    }
}

