//
// Created by stumbo on 18-5-10.
//
#include <iostream>
#include <algorithm>

#include "NodeInstance.h"
#include "ExitInstance.h"
#include "TopoTools.h"

using namespace std;

size_t NodeInstance::serialCount = 0;

/**NWU
 * @param posx coor x according to middle
 * @param posy coor y according to middle
 * @param dir yaw in degree 0-360
 */
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

/**
 * check & adjust the dir to 0-360°
 * @param d
 * @return the adjusted direction
 */
const double & NodeInstance::checkDir(double &d) {
    if (d >= 360.0 || d < 0.0) {
        cout << "[WARNING] you add a direction out of range:" << (int)d << endl;
        d -= 360.0 * (int)(d/360.0);
    }
    return d;
}

void NodeInstance::addUseage(MapCandidate *usedMap, TopoNode *usedAt) {
    nodeUseages.insert({usedMap, usedAt});
}

void NodeInstance::removeUseage(MapCandidate *map2unbind) {
    nodeUseages.erase(map2unbind);
}

/**
 * compare two nodes if it IS ALIKE
 * @param rnode
 * @return is it alike
 */
bool NodeInstance::alike(const NodeInstance & rnode) const {
    const NodeInstance & lnode = *this;

    /**the instance should be added complete*/
    if (!this->addComplete || !rnode.isAddComplete()) {
        cout << "[WARNING] compare nodes before add Complete!!" << endl;
        cout << "plz call completeAdding" << endl;
    }

    /**check if the special msg is the same*/
    if (lnode.extraMsg != rnode.extraMsg) {
        return false;
    }

    /**check the exit numbers*/
    auto & lExits = lnode.exits;
    auto & rExits = rnode.exits;
    if (lExits.size() != rExits.size()) {
        return false;
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
    auto rExitBack = lExits.end() - 1;

    double firstEleRadDif = lExit->getMidRad() - rExit->getMidRad();

    if (firstEleRadDif > exitRadTollerance()) {
        double secondDif = abs(lExitBack->getMidRad() - rExit->getMidRad()) - piTwo;
        if (secondDif > exitRadTollerance()) {
            return false;
        }
        lExit = lExitBack;
    }
    else if (firstEleRadDif < -exitRadTollerance()) {
        double secondDif = abs(lExitBack->getMidRad() - rExit->getMidRad()) - piTwo;
        if (secondDif > exitRadTollerance()) {
            return false;
        }
        rExit = rExitBack;
    }

    for (int i = 0; i < exits.size(); i++) {
        if (!lExit->alike(*rExit)) {
            return false;
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

    return true;
}

NodeInstance::NodeInstance()
        : serialNumber(serialCount)
{
    serialCount++;
}

/**
 * encode the instance to the rosMsg, which need 3 paras about the movement to the node
 * @param arriveAt
 * @param odomX
 * @param odomY
 * @return
 */
topology_map::NewNodeMsgPtr NodeInstance::encode2ROSmsg(unsigned char arriveAt,
                                                        float odomX, float odomY) {
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
    msgPtr->arriveAt = arriveAt;
    return msgPtr;
}

