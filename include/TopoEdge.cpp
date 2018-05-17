//
// Created by stumbo on 18-5-16.
//

#include "TopoEdge.h"
#include "MapCandidate.h"
#include "TopoNode.h"

#include <cmath>

/**
 * change one exit to another exit
 * @attention the node related will be modified too
 * @param oldNode
 * @param newNode
 * @param newGate
 */
void TopoEdge::changeExitTo(TopoNode *const oldNode, TopoNode *const newNode, const uint8_t newGate) {
    if (oldNode == exitB) {
        exitB->disconnectEdge(gateB);
        exitB = newNode;
        gateB = newGate;
        exitB->addEdge(gateB, this);
    } else if (oldNode == exitA) {
        exitA->disconnectEdge(gateA);
        exitA = newNode;
        gateA = newGate;
        exitA->addEdge(gateA, this);
    } else {
        cout << "TopoEdge changeExitTo FAILURE" << endl;
        throw;
    }
}

/**
 * record on the edge that we just move on the node on a direction
 * @param leftnode
 */
void TopoEdge::leaveFromNode(TopoNode *leftnode) {
    if (leftnode == exitA) {
        a2b = true;
    } else if (leftnode == exitB) {
        b2a = true;
    } else {
        cout << "TopoEdge leaveFromNode FAILURE" << endl;
        throw;
    }
}

bool TopoEdge::haveLeftFromNode(TopoNode *leftnode) {
    if (leftnode == exitA) {
        return a2b;
    } else if (leftnode == exitB) {
        return b2a;
    } else {
        cout << "TopoEdge haveLeftFromNode FAILURE" << endl;
        throw;
    }
}

uint16_t TopoEdge::addOdomData(double dis_x, double dis_y, TopoNode * leftNode) {
    using std::abs;
    if (leftNode == exitB) {
        dis_x *= -1.0;
        dis_y *= -1.0;
    }
    else if (leftNode != exitA) {
        cout << "addOdomData no matching Exit FAILURE" << endl;
        throw;
    }
    if (odomAverage != 0) {
        double odomDis = sqrt(odomX * odomX + odomY * odomY);
        if (abs(dis_x - odomX) + abs(dis_y - odomY) > odomDis / 10.0) {
            cout << "edge not match well\nrecorded: " << odomX << " " << odomY
                 << "current :" << odomX << " " << odomY << endl;
        }
    }
    double x = (odomX * odomAverage) + dis_x;
    double y = (odomY * odomAverage) + dis_y;
    odomAverage ++;
    odomX = x / odomAverage;
    odomY = y / odomAverage;
    return odomAverage;
}
