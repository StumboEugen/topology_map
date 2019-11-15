//
// Created by stumbo on 18-5-10.
//
#include <iostream>
#include <algorithm>
#include <cmath>

#include "NodeInstance.h"
#include "ExitInstance.h"
#include "TopoTools.h"

using namespace std;

size_t NodeInstance::serialCount = 1;

/// purest constructor of a empty NodeInstance
/**
 * @param registerSerial if the serial number is registered
 * @see serialNumber
 */
NodeInstance::NodeInstance(bool registerSerial)
{
    if (registerSerial) {
        serialNumber = serialCount++;
    } else {
        serialNumber = SIZE_MAX;
    }
}

/// copy constructor from another NodeInstance
/**
 * @param lIns the copyed node instance
 * @param registerSerial if the serial number is registered
 * @see serialNumber
 */
NodeInstance::NodeInstance(const NodeInstance & lIns, bool registerSerial)
        :exitNums(lIns.exitNums),
         extraMsg(lIns.extraMsg),
         exits(lIns.exits),
         addComplete(lIns.addComplete){
    if (registerSerial) {
        serialNumber = serialCount++;
    }
}

///add an exit to this instance (ENU)
/**
* @param posx the posx relative to the middle of the instance
* @param posy the posy relative to the middle of the instance
* @param dir the outward of the exit 0-360
* @note \b example: instance like 'G', exit is like : (1,1,120) and (0,0,270)
*/
void NodeInstance::addExit(double posx, double posy, double dir) {
    if (addComplete) {
        cout << "[NodeInstance::addExit] you add an exit to a completed node!" << endl;
    }
    dir = checkDir(dir);
    exits.emplace_back(posx, posy, dir);
    exitNums ++;
}

/// the final step of adding is sorting exitInstances
/**
 * @see ExitInstance::operator<()
 */
void NodeInstance::completeAdding() {
    sort(exits.begin(), exits.end());
    exitNums = static_cast<uint8_t>(exits.size());
    addComplete = true;
}

/**
* @param d reference of the dir
* @return the adjusted direction
*/
const double & NodeInstance::checkDir(double &d) {
    if (d >= 360.0 || d < 0.0) {
        d -= 360.0 * floor(d/360.0);
    }
    return d;
}

/// add the map useage to this node instance
/**
 * @param usedMap the MapCandidate (this should be unique)
 * @param usedAt the related TopoNode in the MapCandidate
 */
void NodeInstance::addUseage(MapCandidate *usedMap, TopoNode *usedAt) {
    nodeUseages.insert({usedMap, usedAt});
}

/// remove the map useage form the record
void NodeInstance::removeUseage(MapCandidate *map2unbind) {
    nodeUseages.erase(map2unbind);
}

/// compare another node if they are alike
/**
 * @param rnode the other node
 * @return -1: unlike\n
 * other : the dislocation of the gate NO.\
 * @note
 * \b example: \n
 * that: (-pi, 0, pi/2)\n
 * this: (0, pi/2, pi)\n
 * the return would be 1\n
 * means: gate NO. of \a this needs to +1
 *
 */
int NodeInstance::alike(const NodeInstance & rnode) const {
    const NodeInstance * lnode = this;

    /**the instance should be added complete*/
    if (!this->addComplete || !rnode.isAddComplete()) {
        cout << "[WARNING] compare nodes before add Complete!!" << endl;
        cout << "plz call completeAdding" << endl;
    }

    /**check if the special msg is the same*/
    if (lnode->extraMsg != rnode.extraMsg) {
        return -1;
    }

    /**check the exit numbers*/
    auto & lExits = lnode->exits;
    auto & rExits = rnode.exits;
    if (lExits.size() != rExits.size()) {
        return -1;
    }

    if (lExits.size() == 1) {
        return lExits.begin()->alike(*lExits.begin());
    }

    /**check the pos and dir of all exits
     * because of the marginal effect, we need to compare the in loop sequence
     */
    auto lExit = lExits.begin();
    auto rExit = rExits.begin();
    auto lExitBack = lExits.end() - 1;
    auto rExitBack = rExits.end() - 1;

    double difOri = abs(lExit->getMidRad() - rExit->getMidRad());
    double difL1RB = abs(rExitBack->getMidRad() - lExit->getMidRad() - piTwo);
    double difLBR1 = abs(lExitBack->getMidRad() - rExit->getMidRad() - piTwo);

    int differNum = 0;

    // we only need to change if difOri is not the smallest
    if (difOri > difL1RB || difOri > difLBR1) {
        if (difL1RB < difLBR1) {
            // in this case, consider lE = WS(-pi), rE = WN(+pi)
            differNum = static_cast<int>(lExits.size() - 1);
            rExit = rExitBack;
        } else {
            // in this case, consider lE = WN(+pi), rE = WS(-pi)
            differNum = 1;
            lExit = lExitBack;
        }
    }

    //the old way is not robust:

//    double firstExitRadDif = lExit->getMidRad() - rExit->getMidRad();
//
//    // in this case, consider lE = 359, rE = 1
//    if (firstExitRadDif > exitRadTollerance()) {
//        double secondDif = abs(lExitBack->getMidRad() - rExit->getMidRad()) - piTwo;
//        if (secondDif > exitRadTollerance()) {
//            return false;
//        }
//        lExit = lExitBack;
//    }
//    // in this case, consider lE = 1, rE = 359
//    else if (firstExitRadDif < -exitRadTollerance()) {
//        double secondDif = abs(lExitBack->getMidRad() - rExit->getMidRad()) - piTwo;
//        if (secondDif > exitRadTollerance()) {
//            return false;
//        }
//        rExit = rExitBack;
//    }

    for (int i = 0; i < exits.size(); i++) {
        if (!lExit->alike(*rExit)) {
            return -1;
        }

        lExit++;
        rExit++;

        if (lExit == lExits.end()) {
            lExit = lExits.begin();
        }
        if (rExit == rExits.end()) {
            rExit = rExits.begin();
        }
    }

    return differNum;
}

/// encode the instance to the rosMsg, which need 3 paras about the movement to the node
/**
 * ENU
 * @param arriveAt the arriving at Exit ID of this instance
 * @param odomX the odom movement since last instance in X
 * @param odomY the odom movement since last instance in Y
 * @param yaw the odom turned since last instance in degree (unused)
 * @return the built ROS msg structure
 * @todo split this(ROS related) out of the lib
 */
topology_map::NewNodeMsgPtr
NodeInstance::encode2ROSmsg(unsigned char arriveAt,
                            float odomX, float odomY, float yaw) {
    if (!addComplete) {
        cout << "[NodeInstance::encode2ROSmsg] w: add is not complete!" << endl;
    }
    auto msgPtr = topology_map::NewNodeMsgPtr(new topology_map::NewNodeMsg());
    msgPtr->exitNum = exitNums;
    msgPtr->specialMsg = extraMsg;
    for (const auto & exit: exits) {
        msgPtr->midPosXs.push_back((float &&) exit.getPosX());
        msgPtr->midPosYs.push_back((float &&) exit.getPosY());
        msgPtr->outDirs.push_back((float &&) exit.getOutDir());
    }
    msgPtr->odomX = odomX;
    msgPtr->odomY = odomY;
    msgPtr->odomYaw = yaw;
    msgPtr->arriveAt = arriveAt;
    return msgPtr;
}

/// convert NodeInstance to JSON structure
/**
 * @return
 * ["No"] int serialNO \n
 * ["Extra"] string the index info\n
 * ["Exits"] array of exits
 * @see ExitInstance::toJS()
 */
JSobj NodeInstance::toJS() const {
    JSobj obj;
//    obj["No"] = serialNumber;
    obj["Extra"] = extraMsg;
    auto & exitJS = obj["Exits"];
    for (const auto & exit: exits) {
        exitJS.append(std::move(exit.toJS()));
    }
    return std::move(obj);
}

//topology_map::NewNodeMsgPtr
//NodeInstance::encode2ROSmsg(unsigned char arriveAt, const array<double, 3> & odomMsg) {
//    return encode2ROSmsg(arriveAt, (float) odomMsg[0], (float) odomMsg[1], (float) odomMsg[2]);
//}

///set the pos in global, (from the origin) and the distance moved
/**
 * @param x the global odom in X
 * @param y the global odom in Y
 * @param movedDis the distance moved since the origin
 * @attention movedDis isn't since last instance
 */
void NodeInstance::setGlobalPos(double x, double y, double movedDis) {
    globalX = x;
    globalY = y;
    travelDis = movedDis;
}

/// get the cloest exit of the dir
/**
 * @param midDir in rad
 * @return the closest exit id
 */
gateId NodeInstance::getMidDirClosestExit(double midDir) {
    midDir = topo::fixRad2nppi(midDir);
    double curDif = piTwo;
    double curI = 0;
    for (int i = 0; i < exits.size(); ++i) {
        double dirDif = abs(midDir - exits[i].getMidRad());
        dirDif = min(dirDif, piTwo - dirDif);
        if (dirDif < curDif) {
            curDif = dirDif;
            curI = i;
        }
    }
    return static_cast<gateId>(curI);
}

/// find the closest exit of the given pos
/**
 * @param posx the search pos's x
 * @param posy the search pos's y
 * @return the cloest gate's id
 * @attention for speed, the distance doesn't use sqrt
 */
gateId NodeInstance::figureOutWhichExitItis(double posx, double posy) {
    int ans = 0;
    double dist = abs(posx - exits[0].getPosX()) + abs(posy - exits[0].getPosY());
    for (int i = 1; i < exits.size(); ++i) {
        double newDist = abs(posx - exits[i].getPosX()) + abs(posy - exits[i].getPosY());
        if (newDist < dist) {
            ans = i;
            dist = newDist;
        }
    }
    return static_cast<gateId>(ans);
}

/**
 * @brief find the related TopoNode of given MapCandidate
 * @param givenMap the key MapCandidate you would like to find
 * @return the Node usage in the given MapCandidate, if not found, return nullptr
 */
TopoNode* NodeInstance::getNodeUsageOfGivenMap(MapCandidate* givenMap) const
{
    auto it = nodeUseages.find(givenMap);
    if (it == nodeUseages.end()) {
        cerr << __FILE__ << ":" << __LINE__ << "[ERROR] you find inexist useage in "
                                               "NodeInstance!" << endl;
        return nullptr;
    } else {
        return it->second;
    }
}


