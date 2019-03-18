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

size_t NodeInstance::serialCount = 0;

void NodeInstance::addExit(double posx, double posy, double dir) {
    if (addComplete) {
        cout << "[NodeInstance::addExit] you add an exit to a completed node!" << endl;
    }
    dir = checkDir(dir);
    exits.emplace_back(posx, posy, dir);
    exitNums ++;
}

void NodeInstance::completeAdding() {
    sort(exits.begin(), exits.end());
    exitNums = static_cast<uint8_t>(exits.size());
    addComplete = true;
}

const double & NodeInstance::checkDir(double &d) {
    if (d >= 360.0 || d < 0.0) {
        cout << "[WARNING] you add a direction out of range:" << (int)d << endl;
        d -= 360.0 * floor(d/360.0);
    }
    return d;
}

void NodeInstance::addUseage(MapCandidate *usedMap, TopoNode *usedAt) {
    nodeUseages.insert({usedMap, usedAt});
}

void NodeInstance::removeUseage(MapCandidate *map2unbind) {
    nodeUseages.erase(map2unbind);
    if (nodeUseages.empty()) {
        //TODO need suicide?
    }
}


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
            // in this case, consider lE = 1, rE = 359
            differNum = static_cast<int>(lExits.size() - 1);
            rExit = rExitBack;
        } else {
            // in this case, consider lE = 359, rE = 1
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

NodeInstance::NodeInstance(bool registerSerial)
        : serialNumber(serialCount)
{
    if (registerSerial) {
        serialNumber = serialCount++;
    }
}

/**
 * encode the instance to the rosMsg, which need 3 paras about the movement to the node
 * ENU
 * @param arriveAt
 * @param odomX
 * @param odomY
 * @return
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

JSobj NodeInstance::toJS() const {
    JSobj obj;
    obj["No"] = serialNumber;
    obj["Extra"] = extraMsg;
    auto & exitJS = obj["Exits"];
    for (const auto & exit: exits) {
        exitJS.append(exit.toJS());
    }
    return std::move(obj);
}

NodeInstance::NodeInstance(const NodeInstance & lIns, bool registerSerial)
        :exitNums(lIns.exitNums),
         extraMsg(lIns.extraMsg),
         exits(lIns.exits),
         addComplete(lIns.addComplete){
    if (registerSerial) {
        serialNumber = serialCount++;
    }
}

topology_map::NewNodeMsgPtr
NodeInstance::encode2ROSmsg(unsigned char arriveAt, const array<double, 3> & odomMsg) {
    return encode2ROSmsg(arriveAt, (float) odomMsg[0], (float) odomMsg[1], (float) odomMsg[2]);
}

void NodeInstance::setGlobalPos(double x, double y, double movedDis) {
    globalX = x;
    globalY = y;
    travelDis = movedDis;
}

