//
// Created by stumbo on 18-5-16.
//

#include "TopoEdge.h"
#include "MapCandidate.h"
#include "TopoNode.h"
#include "NodeInstance.h"

#include <cmath>

/**
 * copy constructor from another TopoEdge.
 *
 * copy most information, but Node of \b this is different from \b that, and \b this would be
 * in another MapCandidate
 * @warning if a given TopoNode is nullptr, it will throw
 * @attention make sure that ea and eb's relationship is the same as the old ones
 * @param that the origin TopoEdge to copy from
 * @param ea new node A
 * @param eb new node B
 */
TopoEdge::TopoEdge(const TopoEdge &that, TopoNode *const ea, TopoNode *const eb)
        :exitA(ea),
         exitB(eb),
         gateA(that.gateA),
         gateB(that.gateB),
         b2aMoved(that.b2aMoved),
         a2bMoved(that.a2bMoved),
         odomX(that.odomX),
         odomY(that.odomY),
         yawOdom(that.yawOdom),
         odomCount(that.odomCount)
{
    if (exitA != nullptr && exitB != nullptr) {
        registerAtNodes();
    } else {
        cerr << "TopoEdge construct with nullptr node!" << endl;
        throw;
    }
}

/**
 * constructor of a empty Edge.
 *
 * @param ea the node left from
 * @param ga the gateNO left from
 * @param eb the node arrive at
 * @param gb the gateNo arrive at
 *
 * @attention ea is left from and eb is arrived at, you may not confuse them
 * @warning if a given TopoNode is nullptr, it will throw
 */
TopoEdge::TopoEdge(TopoNode *const ea, uint8_t ga, TopoNode *const eb, uint8_t gb)
        :exitA(ea),
         exitB(eb),
         gateA(ga),
         gateB(gb),
         b2aMoved(false),
         a2bMoved(true),
         odomX(0.0),
         odomY(0.0),
         yawOdom(0.0),
         odomCount(0)
{
    if (exitA != nullptr && exitB != nullptr) {
        registerAtNodes();
    } else {
        cerr << "TopoEdge construct with nullptr node!" << endl;
        throw;
    }
}


/**
 * get a connected TopoNode from another TopoNode.
 *
 * @param node the opposite TopoNode of the desired TopoNode
 * @warning if the node given doesn't connect to the edge, it will throw
 * @return the another node of the input TopoNode
 */
TopoNode *const TopoEdge::getAnotherNode(TopoNode *node) const  {
    if (node == exitA) {
        return exitB;
    }
    if (node == exitB) {
        return exitA;
    }
    std::cout << "TopoEdge another node FAILURE" << std::endl;
    throw;
}

/**
 * get a gateNO from another TopoNode
 *
 * @param node the other TopoNode of the required gateNO
 * @return the gateNO of the opposite side of the input TopoNode
 */
unsigned char TopoEdge::getAnotherGate(TopoNode *node) const {
    if (node == exitA) {
        return gateB;
    }
    if (node == exitB) {
        return gateA;
    }
    std::cout << "TopoEdge another gate FAILURE" << std::endl;
    throw;
}

/**
 * change gate's NO of required TopoNode
 * @param node the gateNO changed TopoNode
 * @warning if the node is illegial, it will throw
 * @param newGateID the new gate ID assigned
 * @note the newGateID isn't checked if it is legial
 */
void TopoEdge::resetGateNO(TopoNode *node, gateId newGateID)  {
    if (node == exitA) {
        gateA = newGateID;
    } else if (node == exitB) {
        gateB = newGateID;
    } else {
        std::cerr << "TopoEdge resetGateNO FAILURE" << std::endl;
        throw;
    }
}

/**
 * change one exit to another exit
 * @param oldNode the old TopoNode 2 detach
 * @param newNode the new TopoNode 2 connect
 * @param newGate the new gateNO of the new TopoNode
 * @attention the node related will be modified too
 * @warning if the TopoNode given is illegial, it will throw
 */
void TopoEdge::changeExitTo(
        TopoNode *const oldNode, TopoNode *const newNode, const uint8_t newGate) {
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
        cerr << "[TopoEdge::changeExitTo] FAILURE" << endl;
        throw;
    }
}

/**
 * add new odom data to this edge, and get a fix coefficient from the observation.
 * @param dis_x the new odom data X-axis ENU
 * @param dis_y the new odom data Y-axis ENU
 * @param yaw turn yaw, conter clockwise is +, in RAD (unused yet, may abandon)
 * @param leftNode the left Node to make sure the correct of positive/negative
 * @return the fix confience calculated from the observation
 * @warning if the leftNode is illegial, it will throw
 */
double TopoEdge::addOdomData(double dis_x, double dis_y, double yaw, TopoNode *leftNode) {
    using std::abs;
    if (leftNode == exitB) {
        dis_x *= -1.0;
        dis_y *= -1.0;
    }
    else if (leftNode != exitA) {
        cerr << "addOdomData no matching Exit FAILURE" << endl;
        throw;
    }

    double fixCoe = 1.0;

    if (odomCount != 0) {
        double odomRecorded = sqrt(odomX * odomX + odomY * odomY);
        double distGot = sqrt(dis_x * dis_x + dis_y * dis_y);
        fixCoe = odomRecorded - distGot;
        fixCoe *= fixCoe;
        double conv = convEdgePerMeter * odomRecorded * (odomCount + 1.0) / odomCount;
        fixCoe /= conv;
        if (fixCoe > 16) {
//            return 0.0; TODO
        }
        fixCoe = exp (-0.5 * fixCoe);

//        if (abs(dis_x - odomX) + abs(dis_y - odomY) > odomRecorded / 10.0) {
//            cout << "[TopoEdge::addOdomData]edge not match well\n"
//                    "recorded: " << odomX << " " << odomY
//                 << "current :" << dis_x << " " << dis_y << endl;
//        }
    }
    double x = (odomX * odomCount) + dis_x;
    double y = (odomY * odomCount) + dis_y;
    double turn = (yawOdom * odomCount) + yaw;
    odomCount ++;
    odomX = x / odomCount;
    odomY = y / odomCount;
    yawOdom = turn / odomCount;
    return fixCoe;
}

/// set odom data violently ENU
void TopoEdge::setOdomDataDirectly(double x, double y, double yaw) {
    odomCount = 1;
    odomX = x;
    odomY = y;
    yawOdom = yaw;
}

/**
 * get the odom data.
 * because A2B and B2A is different, we need the origin TopoNode
 * @param oriNode the origin TopoNode of the odom data
 * @return the odominfo in array <odomX, odomY, yawOdom> ENU
 */
array<double, 3> TopoEdge::getOdomData(TopoNode *oriNode) {
    if (oriNode == exitA) {
        return {odomX, odomY, yawOdom};
    } else if (oriNode == exitB) {
        return {-odomX, -odomY, yawOdom};
    } else {
        cerr << "TopoEdge::getOdomData no matching Exit FAILURE" << endl;
        throw;
    }
}

/**
 * convert the TopoEdge to JSON structure
 * @return
 * ["Ea"] int the serial number of the exitA TopoNode 's corresponding NodeInstance
 * ["Eb"] int the serial number of the exitB TopoNode 's corresponding NodeInstance
 * ["Ga"] int the gateNO at exitA TopoNode
 * ["Gb"] int the gateNO at exitB TopoNode
 * ["Ox"] float odom data X-axis
 * ["Oy"] float odom data Y-axis
 * ["Oa"] int the ammount of observation
 * ["yaw"] the turn data
 */
JSobj TopoEdge::toJS() const {
    JSobj obj;
    obj["Ea"] = exitA->getInsCorrespond()->getSerialNumber();
    obj["Eb"] = exitB->getInsCorrespond()->getSerialNumber();
    obj["Ga"] = gateA;
    obj["Gb"] = gateB;
    obj["Ox"] = odomX;
    obj["yaw"] = yawOdom;
    obj["Oy"] = odomY;
    obj["Oa"] = odomCount;
    return std::move(obj);
}

/**
 * register edge info at TopoNode
 */
void TopoEdge::registerAtNodes() {
//    if (exitA != nullptr && exitB != nullptr) {
        exitA->addEdge(gateA, this);
        exitB->addEdge(gateB, this);
//    }
}

void TopoEdge::leaveFromNode(TopoNode *leftnode) {
    if (leftnode == exitA) {
        a2bMoved = true;
    } else if (leftnode == exitB) {
        b2aMoved = true;
    } else {
        cerr << "TopoEdge leaveFromNode FAILURE" << endl;
        throw;
    }
}

bool TopoEdge::haveLeftFromNode(TopoNode *leftnode) {
    if (leftnode == exitA) {
        return a2bMoved;
    } else if (leftnode == exitB) {
        return b2aMoved;
    } else {
        cerr << "TopoEdge haveLeftFromNode FAILURE" << endl;
        throw;
    }
}
