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
 * represent a node in MapCandidate.
 * The real structure of real world are stroed in NodeInstance.
 * There might be sreval related NodeInstances,
 * because we may arrive at a same place at different time.
 * @note the main related NodeInstance is the back of the TopoNode::relatedInses
 * @attention no information about the MapCandidate is stored here, so we can't find
 * MapCandidate from TopoNode
 */
class TopoNode {

public:

    // create a TopoNode according to the nodeInstance
    explicit TopoNode(NodeInstance * nodeInstance);

    // copy constructor from another TopoNode
    explicit TopoNode(const TopoNode *that);

    // rotate the exits' ID
    void ringRotate(int diff);

    // a simple tool to find which gateId is the topoNode at
    gateId gateIdOfTheTopoEdge(TopoEdge *targetEdge) const;

    // convert the TopoNode to JSON structure
    JSobj toJS() const;

    /// check if the exit is empty (nullptr) (hasn't moved through)
    bool isExitEmpty(gateId number) const {
        return edgeConnected[number] == nullptr;
    }

    /// set the exit at index (connect the TopoNode and TopoEdge)
    /// @warning not check if number is legal
    void addEdge(gateId number, TopoEdge* edge) {
        edgeConnected[number] = edge;
    }

    /// get the edge at specific gate
    /// @warning not check if number is legal
    TopoEdge *const getEdge(gateId number) const {
        return edgeConnected[number];
    }

    /// get the main NodeInstance, the back of TopoNode::relatedInses
    /// @note it is the same as getTheLastRelatedIns(), because we need backwards compatible
    NodeInstance *const getInsCorrespond() const {
        return relatedInses.back();
    }

    /// get the back of TopoNode::relatedInses
    NodeInstance *const getTheLastRelatedIns() const {
        return relatedInses.back();
    }

    /// push a new related NodeInstance to the back of TopoNode::relatedInses
    void addNewRelatedIns(NodeInstance * ins) {
        relatedInses.push_back(ins);
    }

    const std::vector<NodeInstance *> & getRelatedInses() const {
        return relatedInses;
    }

    /// mannual break a connection with a TopoEdge
    void disconnectEdge(gateId number) {
        edgeConnected[number] = nullptr;
    }

    const std::vector<TopoEdge *> & getEdgeConnected() const {
        return edgeConnected;
    }

    // in old version there are some bit operation
    void cleanFlags() {
        tempFlags = 0;
        assistPtr = nullptr;
    }

    // in old version there are some bit operation
    void setFlag(uint8_t index, bool set = true) {
        if (index > 32) return;
        if (set) {
            tempFlags |= 1u << index;
        } else {
            tempFlags &= ~(1u << index);
        }
    }

    // in old version there are some bit operation
    bool chkFlag(uint8_t index) {
        if (index > 32) return false;
        return (tempFlags & (1u << index)) != 0;
    }

    /// currently for BFS in UI
    void *getAssistPtr() const {
        return assistPtr;
    }

    /// currently for BFS in UI
    void setAssistPtr(void *helperPtr) {
        TopoNode::assistPtr = helperPtr;
    }

public: // member
    /// a tool to help cloning maps
    /// @see MapCandidate::MapCandidate(const MapCandidate &)
    TopoNode * clonedTo;

private: // member
    /// a list of related NodeInstace, in the order of time.
    /// which means that the back of relatedInses is the last observation of the node
    std::vector<NodeInstance *> relatedInses;

    /// edges connected at this TopoNode
    /// @note the index is following the back of TopoNode::relatedInses, so this may change
    std::vector<TopoEdge*> edgeConnected;

    // a tool to help iterating the map
    uint32_t tempFlags = 0;

    // a tool to help iterating the map
    void * assistPtr = nullptr;
};

#endif //TOPOLOGY_MAP_TOPONODE_H
