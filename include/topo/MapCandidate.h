//
// Created by stumbo on 18-5-14.
//

#ifndef TOPOLOGY_MAP_MAPCANDIDATE_H
#define TOPOLOGY_MAP_MAPCANDIDATE_H

#include <memory>
#include <vector>
#include <set>
#include <list>
#include <ostream>
#include "TopoTools.h"

class TopoPath;

/**
 * @brief represent a possible option of the real topo structure map.
 *
 * includes TopoEdge s and TopoNode s. a MapCandidate is stored and arranged by
 * MapCollection.
 */
class MapCandidate {
public:
    // generate a MapCandidate from the JSON info
    MapCandidate(const std::vector<NodeInstance*> & nodeInses, const JSobj & JSinfo);

    // generate a MapCandidate that only has a TopoNode
    explicit MapCandidate(NodeInstance *);

    // copy constructor
    MapCandidate(const MapCandidate&);

    // set the robot's state in this MapCandidate to leave at exit
    void setLeaveFrom(gateId exit);

    // inform the MapCandidate the robot arrives at a NodeInstance
    bool arriveAtNode(NodeInstance *instance, gateId arriveAt, double dis_x,
            double dis_y, double yaw);

    // clone another MapCandidate from the origin one with a loop-closure
    MapCandidate *const
    arriveAtSimiliar(TopoNode *arriveNode, gateId arriveGate, int gateDiff);

    // clear all assistance flags and ptrs of the TopoNodes
    void cleanAllNodeFlagsAndPtr();

    // create a new node into the nodes set
    TopoNode *const addNewNode(NodeInstance * instance);

    // create a new edge into the nodes set
    TopoEdge *const addNewEdge(TopoNode * ea, gateId ga, TopoNode * eb, gateId gb);

    // erase a TopoNode (destroy) in the nodes set
    void removeNode(TopoNode * node2remove);

    // convert MapCandidate to JSON structure
    JSobj toJS() const;

    // the destructor destroy all TopoEdge and TopoNode.
    ~MapCandidate();

    // remove all usages of the TopoNode s in related NodeInstances
    void removeUseages();

    // update the confidence
    void xConfidence(double coe);

    // calculate the confidence of this map (not normalized)
    double getConfidence(double experienceCountK) const;

    double getConfidencePURE() const {
        return confidence;
    }

    size_t getFullEdgeNumber() const {
        return fullEdgeNumber;
    }

    const unsigned long getNodeNum() const {
        return nodes.size();
    }

    const unsigned long getEdgeNum() const {
        return edges.size();
    }

    const std::set<TopoNode *> & getNodes() const {
        return nodes;
    }

    const std::set<TopoEdge *> & getEdges() const {
        return edges;
    }

    bool isJustMovedOnKnownEdge() const {
        return lastEdgeIsOldEdge;
    }

    TopoNode *const getOneTopoNode() {
        return nodes.begin().operator*();
    }

    void addNodeDirectly(TopoNode * node) {
        nodes.insert(node);
    }

    void addEdgeDirectly(TopoEdge * edge) {
        edges.insert(edge);
    }

    TopoNode *getCurrentNode() const {
        return currentNode;
    }

private: // function

    // consider the new node instance as a new node in the map
    void arriveNewNode(NodeInstance *instance, gateId arriveAt);

private: // member

    /// the container of the TopoNode
    /// @todo using vector might is better, (remember to delete the new TopoNode after copy)
    std::set<TopoNode *> nodes;

    /// the container of the TopoNode
    /// @todo using vector might is better, (remember to delete the new TopoNode after copy)
    std::set<TopoEdge *> edges;

    /// the confidence of this map (the KEY of this project), begin with 1.0
    double confidence = 1.0;

    /**
     * @brief the TopoNode state of robot
     *
     * on edge -> the node just left
     *
     * on node -> current node
     */
    TopoNode * currentNode;

    /**
     * @brief the TopoEdge state of robot
     *
     * nullptr means moving on an edge have never been to
     *
     * on edge -> current edge
     *
     * on node -> last edge
     * @attention THIS CAN BE NULL!
     * */
    TopoEdge * currentEdge;

    /// if the last edge is an old edge (means possible conflict)
    bool lastEdgeIsOldEdge;

    // /the exit ID that you left from the last node
    uint8_t leaveFrom;

    /// the sum of all nodes' edges, to determine the close situation
    /// @deprecated the new sort way uses BAYS, not this
    size_t fullEdgeNumber;
};



#endif //TOPOLOGY_MAP_MAPCANDIDATE_H
