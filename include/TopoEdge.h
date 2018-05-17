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
    TopoEdge(TopoNode *const ea, uint8_t ga, TopoNode *const eb, uint8_t gb)
            :exitA(ea),
             exitB(eb),
             gateA(ga),
             gateB(gb),
             b2a(false),
             a2b(true),
             odomX(0.0),
             odomY(0.0),
             odomAverage(0)
    //TODO set moved from a2b b2a
    {}

    /**
     * copy from another edge, but the nodePtr should be different
     * @param edge oriEdge to copy from
     * @param ea new node A
     * @param eb new node B
     */
    TopoEdge(const TopoEdge& edge, TopoNode *const ea, TopoNode *const eb)
            :exitA(ea),
             exitB(eb),
             gateA(edge.gateA),
             gateB(edge.gateB),
             b2a(edge.b2a),
             a2b(edge.a2b),
             odomX(edge.odomX),
             odomY(edge.odomY),
             odomAverage(edge.odomAverage)
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

    void leaveFromNode(TopoNode *leftnode);

    bool haveLeftFromNode(TopoNode *leftnode);

    uint16_t addOdomData(double dis_x, double dis_y, TopoNode * leftNode);

    double getOdomX() const {
        return odomX;
    }

    double getOdomY() const {
        return odomY;
    }

private:
    TopoNode * exitA;
    TopoNode * exitB;
    uint8_t gateA;
    uint8_t gateB;
    bool a2b;
    bool b2a;
    double odomX;
    double odomY;
    uint16_t odomAverage;
};


#endif //TOPOLOGY_MAP_TOPOEDGE_H
