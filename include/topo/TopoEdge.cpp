//
// Created by stumbo on 18-5-16.
//

#include "TopoEdge.h"
#include "MapCandidate.h"
#include "TopoNode.h"
#include "NodeInstance.h"

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
        cout << "[TopoEdge::changeExitTo] FAILURE" << endl;
        throw;
    }
}

/**
 * record on the edge that we just move on the node on a direction
 * @param leftnode
 */
void TopoEdge::leaveFromNode(TopoNode *leftnode) {
    if (leftnode == exitA) {
        a2bMoved = true;
    } else if (leftnode == exitB) {
        b2aMoved = true;
    } else {
        cout << "TopoEdge leaveFromNode FAILURE" << endl;
        throw;
    }
}

/**
 * @return if moved on this edge leaving from another node in the history
 */
bool TopoEdge::haveLeftFromNode(TopoNode *leftnode) {
    if (leftnode == exitA) {
        return a2bMoved;
    } else if (leftnode == exitB) {
        return b2aMoved;
    } else {
        cout << "TopoEdge haveLeftFromNode FAILURE" << endl;
        throw;
    }
}

/**
 * add odom data to this edge
 * @return the experience on this node
 */
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

/**
 * copy from another edge, but the nodePtr should be different
 * @param edge oriEdge to copy from
 * @param ea new node A
 * @param eb new node B
 */
TopoEdge::TopoEdge(const TopoEdge &edge, TopoNode *const ea, TopoNode *const eb)
        :exitA(ea),
         exitB(eb),
         gateA(edge.gateA),
         gateB(edge.gateB),
         b2aMoved(edge.b2aMoved),
         a2bMoved(edge.a2bMoved),
         odomX(edge.odomX),
         odomY(edge.odomY),
         odomAverage(edge.odomAverage)
{}

TopoEdge::TopoEdge(TopoNode *const ea, uint8_t ga, TopoNode *const eb, uint8_t gb)
        :exitA(ea),
         exitB(eb),
         gateA(ga),
         gateB(gb),
         b2aMoved(false),
         a2bMoved(true),
         odomX(0.0),
         odomY(0.0),
         odomAverage(0)
//TODO set moved from a2bMoved b2aMoved
{}

JSobj TopoEdge::toJS() const {
    JSobj obj;
    obj["Ea"] = exitA->getInstanceCorresponding()->getSerialNumber();
    obj["Eb"] = exitB->getInstanceCorresponding()->getSerialNumber();
    obj["Ga"] = gateA;
    obj["Gb"] = gateB;
    obj["Ox"] = odomX;
    obj["Oy"] = odomY;
    obj["Oa"] = odomAverage;
    return std::move(obj);
}
