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
    MapCandidate(const NodeInstance *);
    MapCandidate(const MapCandidate&);
    void setLeaveFrom(uint8_t exit);
    void arriveAtNode(const NodeInstance *instance, uint8_t arriveAt);
    MapCandidate *const arriveAtSimiliar(TopoNode *arriveNode, uint8_t arriveGate);

    const unsigned getFullEdigeNumber() const {
        return fullEdgeNumber;
    }

    const unsigned long getNodeNum() const {
        return nodes.size();
    }

    const unsigned long getEdgeNum() const {
        return edges.size();
    }

    TopoNode *const addNewNode(const NodeInstance * instance);

    TopoEdge *const addNewEdge(TopoNode * ea, uint8_t ga, TopoNode * eb, uint8_t gb);

    void removeNode(TopoNode * node2remove) {
        delete node2remove;
        nodes.erase(node2remove);
    }

    void suicide();

private:
    void arriveNewNode(const NodeInstance *instance, uint8_t arriveAt);
    set<TopoNode *> nodes;
    set<TopoEdge *> edges;
    TopoNode * currentNode;
    /**CAN BE NULL
     * means moving on an edge have never been to*/
    TopoEdge * currentEdge;
    bool justArriveNew;
    uint8_t leaveFrom;
    unsigned fullEdgeNumber;
};

class TopoEdge {

public:
    TopoEdge(TopoNode *const ea, uint8_t ga,TopoNode *const eb, uint8_t gb)
            :exitA(ea),
             exitB(eb),
             gateA(ga),
             gateB(gb)
    {}

    TopoNode *const getAnotherNode(TopoNode* node) const {
        if (node == exitA) {
            return exitB;
        }
        if (node == exitB) {
            return exitA;
        }
        cout << "TopoEdge another node FAILURE" << endl;
        throw;
    }

    uint8_t getAnotherGate(TopoNode* node) const {
        if (node == exitA) {
            return gateB;
        }
        if (node == exitB) {
            return gateA;
        }
        cout << "TopoEdge another gate FAILURE" << endl;
        throw;
    }

    TopoNode *const getExitA() const {
        return exitA;
    }

    TopoNode *const getExitB() const {
        return exitB;
    }

    const uint8_t getGateA() const {
        return gateA;
    }

    const uint8_t getGateB() const {
        return gateB;
    }

    void changeExitTo(TopoNode * oldNode, TopoNode * newNode, uint8_t newGate);

private:
    TopoNode * exitA;
    TopoNode * exitB;
    uint8_t gateA;
    uint8_t gateB;
};

class TopoNode {

public:
    explicit TopoNode(const NodeInstance *const nodeInstance):
            corresponding(nodeInstance),
            edgeConnected(nodeInstance->getExitNums(), nullptr),
            clonedTo(nullptr)
    {
        if (!nodeInstance->isAddComplete()) {
            cout << "bind TopoNode before add complete!" << endl;
        }
    }

    bool isExitEmpty(uint8_t number) const {
        return edgeConnected[number] == nullptr;
    }

    void addEdge(uint8_t number, TopoEdge* edge) {
        edgeConnected[number] = edge;
    }

    TopoEdge *const getEdge(uint8_t number) const {
        return edgeConnected[number];
    }

    const NodeInstance *const getInstance() const {
        return corresponding;
    }

    void disconnectEdge(uint8_t number) {
        edgeConnected[number] = nullptr;
    }

    TopoNode * clonedTo;

private:
    const NodeInstance *const corresponding;
    vector<TopoEdge*> edgeConnected;
};


#endif //TOPOLOGY_MAP_MAPCANDIDATE_H
