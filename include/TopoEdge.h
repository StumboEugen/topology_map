//
// Created by stumbo on 18-5-16.
//

#ifndef TOPOLOGY_MAP_TOPOEDGE_H
#define TOPOLOGY_MAP_TOPOEDGE_H

#include <iostream>

using std::cout;
using std::endl;

class TopoNode;

/**
 * represent an edge in map candidate
 */
class TopoEdge {

public:
    TopoEdge(TopoNode * ea, uint8_t ga, TopoNode * eb, uint8_t gb);

    TopoEdge(const TopoEdge& edge, TopoNode * ea, TopoNode * eb);

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
