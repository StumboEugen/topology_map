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

using namespace std;

/**
 * represent a possibility of the real topo structure map
 * using TopoEdges and TopoNodes
 */
class MapCandidate {
public:
    MapCandidate(const std::vector<NodeInstance*> & nodeInses, const JSobj & JSinfo);
    explicit MapCandidate(NodeInstance *);
    MapCandidate(const MapCandidate&);
    void setLeaveFrom(gateId exit);
    bool arriveAtNode(NodeInstance *const instance, gateId arriveAt, const double dis_x,
                          const double dis_y, const double yaw);
    MapCandidate *const arriveAtSimiliar(TopoNode *arriveNode, gateId arriveGate);

    size_t getFullEdgeNumber() const {
        return fullEdgeNumber;
    }

    const unsigned long getNodeNum() const {
        return nodes.size();
    }

    const unsigned long getEdgeNum() const {
        return edges.size();
    }

    const set<TopoNode *> & getNodes() const {
        return nodes;
    }

    const set<TopoEdge *> & getEdges() const {
        return edges;
    }

    bool isJustMovedOnKnownEdge() const {
        return lastEdgeIsOldEdge;
    }

    void setPosInList(mapPosInList listPos) {
        posInList = listPos;
    }

    mapPosInList getPosInList() {
        return posInList;
    }

    void cleanAllNodeFlagsAndPtr();

    TopoNode *const getOneTopoNode() {
        return nodes.begin().operator*();
    }

    void addNodeDirectly(TopoNode * node) {
        nodes.insert(node);
    }

    void addEdgeDirectly(TopoEdge * edge) {
        edges.insert(edge);
    }

    TopoNode *const addNewNode(NodeInstance * instance);

    TopoEdge *const addNewEdge(TopoNode * ea, gateId ga, TopoNode * eb, gateId gb);

    void removeNode(TopoNode * node2remove);

    JSobj toJS() const;

    ~MapCandidate();

    void removeUseages();

private:
    void arriveNewNode(NodeInstance *instance, gateId arriveAt);
    set<TopoNode *> nodes;
    set<TopoEdge *> edges;

    /**
     * on edge -> the node just leave
     *
     * on node -> current node
     */
    TopoNode * currentNode;

    /**CAN BE NULL!
     *
     * nullptr means moving on an edge have never been to
     *
     * on edge -> current edge
     *
     * on node -> last edge*/
    TopoEdge * currentEdge;

    // last edge might be just made
    bool lastEdgeIsOldEdge;

    //the gate ID that you leave from the last node
    uint8_t leaveFrom;

    //the sum of all nodes' edges, to determine the close situation
    size_t fullEdgeNumber;

    //to record the place in the mapCollection std::list<>
    mapPosInList posInList;
};



#endif //TOPOLOGY_MAP_MAPCANDIDATE_H
