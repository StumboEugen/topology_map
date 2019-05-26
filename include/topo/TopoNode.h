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

    TopoNode(const TopoNode *that);

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

    NodeInstance *const getInsCorrespond() const {
        return relatedInses.back();
    }

    NodeInstance *const getTheLastRelatedIns() const {
        return relatedInses.back();
    }

    void addNewRelatedIns(NodeInstance * ins) {
        relatedInses.push_back(ins);
    }

    const vector<NodeInstance *> & getRelatedInses() const {
        return relatedInses;
    }

    void disconnectEdge(gateId number) {
        edgeConnected[number] = nullptr;
    }

    const vector<TopoEdge *> & getEdgeConnected() const {
        return edgeConnected;
    }

    //tool member help cloning maps
    TopoNode * clonedTo;

    JSobj toJS() const;

    void cleanFlags() {
        tempFlags = 0;
        assistPtr = nullptr;
    }

    void setFlag(uint8_t index, bool set = true) {
        if (index > 32) return;
        if (set) {
            tempFlags |= 1u << index;
        } else {
            tempFlags &= ~(1u << index);
        }
    }

    bool chkFlag(uint8_t index) {
        if (index > 32) return false;
        return (tempFlags & (1u << index)) != 0;
    }

    void *getAssistPtr() const {
        return assistPtr;
    }

    void setAssistPtr(void *helperPtr) {
        TopoNode::assistPtr = helperPtr;
    }

private:

    /// a list of related inses in the order of time
    std::vector<NodeInstance *> relatedInses;

//    ///the corresponded instance
//    NodeInstance *const corresponding;

    ///edges connected at this TopoNode
    std::vector<TopoEdge*> edgeConnected;

    ///to help iterate the map
    uint32_t tempFlags = 0;

    void * assistPtr = nullptr;
};

#endif //TOPOLOGY_MAP_TOPONODE_H
