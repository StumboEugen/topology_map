//
// Created by stumbo on 18-5-16.
//

#ifndef TOPOLOGY_MAP_TOPOEDGE_H
#define TOPOLOGY_MAP_TOPOEDGE_H

#include <iostream>
#include "TopoTools.h"

class TopoNode;

/**
 * represent an edge in MapCandidate.
 *
 */
class TopoEdge {

public:
    // constructor of a empty Edge
    TopoEdge(TopoNode * ea, uint8_t ga, TopoNode * eb, uint8_t gb);

    // copy constructor from another TopoEdge.
    TopoEdge(const TopoEdge& that, TopoNode * ea, TopoNode * eb);

    // get a connected TopoNode from another TopoNode
    TopoNode *const getAnotherNode(TopoNode* node) const;

    // get a gateNO from another TopoNode
    unsigned char getAnotherGate(TopoNode* node) const;

    // change gate's NO
    void resetGateNO(TopoNode *node, gateId newGateID);

    // change one exit to another exit
    void changeExitTo(TopoNode * oldNode, TopoNode * newNode, unsigned char newGate);

    // add new odom data to this edge, and get a fix coefficient from the observation.
    double addOdomData(double dis_x, double dis_y, double yaw, TopoNode *leftNode);

    // set odom data violently
    void setOdomDataDirectly(double x, double y, double yaw);

    // get the odom data.
    std::array<double, 3> getOdomData(TopoNode *oriNode);

    // convert the TopoEdge to JSON structure
    JSobj toJS() const;

    TopoNode *const getNodeA() const {
        return exitA;
    }

    TopoNode *const getNodeB() const {
        return exitB;
    }

    const unsigned char getGateA() const {
        return gateA;
    }

    const unsigned char getGateB() const {
        return gateB;
    }

    double getOdomX() const {
        return odomX;
    }

    double getOdomY() const {
        return odomY;
    }

    double getYawOdom() const {
        return yawOdom;
    }

    // set has left from node (unused yet)
    void leaveFromNode(TopoNode *leftnode);

    // check if have left from node (unused yet)
    bool haveLeftFromNode(TopoNode *leftnode);

private:
    // register edge info at TopoNode
    void registerAtNodes();

    /// the connected TopoNode A
    TopoNode * exitA;

    /// the connected TopoNode B
    TopoNode * exitB;

    /// the exitNO at TopoNode A
    uint8_t gateA;

    /// the exitNO at TopoNode B
    uint8_t gateB;

    /// if a2b has moved (unused yet)
    bool a2bMoved;

    /// if b2a has moved (unused yet)
    bool b2aMoved;

    /// odom data at X-axis
    double odomX;

    /// odom data at Y-axis
    double odomY;

    /// odom data in turnning in RAD (unused yet)
    double yawOdom;

    /// the amount of odom data observed
    uint16_t odomCount;
};


#endif //TOPOLOGY_MAP_TOPOEDGE_H
