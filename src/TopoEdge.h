//
// Created by stumbo on 18-5-16.
//

#ifndef TOPOLOGY_MAP_TOPOEDGE_H
#define TOPOLOGY_MAP_TOPOEDGE_H

#include <iostream>

using std::cout;
using std::endl;

class TopoNode;

class TopoEdge {

public:
    TopoEdge(TopoNode *const ea, unsigned char ga, TopoNode *const eb, unsigned char gb)
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

    unsigned char getAnotherGate(TopoNode* node) const {
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

    const unsigned char getGateA() const {
        return gateA;
    }

    const unsigned char getGateB() const {
        return gateB;
    }

    void changeExitTo(TopoNode * oldNode, TopoNode * newNode, unsigned char newGate);

private:
    TopoNode * exitA;
    TopoNode * exitB;
    uint8_t gateA;
    uint8_t gateB;
};


#endif //TOPOLOGY_MAP_TOPOEDGE_H
