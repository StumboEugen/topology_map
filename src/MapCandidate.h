//
// Created by stumbo on 18-5-14.
//

#ifndef TOPOLOGY_MAP_MAPCANDIDATE_H
#define TOPOLOGY_MAP_MAPCANDIDATE_H

#include <memory>
#include <vector>
#include <set>
#include <ostream>

#include "NodeInstance.h"

using namespace std;

class TopoNode;
class TopoEdge;

class MapCandidate {
public:
    MapCandidate(const MapCandidate&);
    void setLeaveFrom(uint8_t exit);
    void arriveAtNode(NodeInstance *instance, uint8_t arriveAt);
    MapCandidate* arriveAtSimiliar(TopoNode *arriveNode, uint8_t arriveGate);

    const unsigned getFullEdigeNumber() const {
        return fullEdgeNumber;
    }

    const unsigned long getNodeNum() const {
        return nodes.size();
    }

    const unsigned long getEdgeNum() const {
        return edges.size();
    }

    void suicide();

private:
    void createNewNode(NodeInstance* instance, uint8_t arriveAt);
    set<TopoNode*> nodes;
    set<TopoEdge*> edges;
    TopoNode * currentNode;
    /**CAN BE NULL*/
    TopoEdge * currentEdge;
    bool justArriveNew = false;
    uint8_t leaveFrom;
    unsigned fullEdgeNumber = 0;
};

class TopoNode {
public:
    TopoNode(const TopoNode&);
    explicit TopoNode(NodeInstance* nodeInstance):
            corresponding(nodeInstance),
            edgeConnected(nodeInstance->getExitNums(), nullptr)
    {
        if (!nodeInstance->isAddComplete()) {
            cout << "bind TopoNode before add complete!" << endl;
        }
    }

    bool isExitEmpty(uint8_t number) {
        return edgeConnected[number] == nullptr;
    }

    void addEdge(uint8_t number, TopoEdge* edge) {
        edgeConnected[number] = edge;
    }

    TopoEdge * getEdge(uint8_t number) const {
        return edgeConnected[number];
    }

    const NodeInstance* getCorrespondInstance() const {
        return corresponding;
    }

private:
    NodeInstance* corresponding;
    vector<TopoEdge*> edgeConnected;
};

class TopoEdge {
public:
    TopoEdge(const TopoEdge&);
    TopoEdge(TopoNode * ea, uint8_t ga, TopoNode * eb, uint8_t gb)
            :exitA(ea),
             exitB(eb),
             gateA(ga),
             gateB(gb)
    {}

    TopoNode* getAnotherNode(TopoNode* node) {
        if (node == exitA) {
            return exitB;
        }
        if (node == exitB) {
            return exitA;
        }
        cout << "TopoEdge another node FAILURE" << endl;
        throw;
    }

    uint8_t getAnotherGate(TopoNode* node) {
        if (node == exitA) {
            return gateB;
        }
        if (node == exitB) {
            return gateA;
        }
        cout << "TopoEdge another gate FAILURE" << endl;
        throw;
    }
private:
    TopoNode * exitA;
    TopoNode * exitB;
    uint8_t gateA;
    uint8_t gateB;
};


#endif //TOPOLOGY_MAP_MAPCANDIDATE_H
