//
// Created by stumbo on 18-5-10.
//
#include <iostream>
#include <algorithm>

#include "NodeInstance.h"
#include "ExitInstance.h"

using namespace std;

void NodeInstance::addExit(double posx, double posy, double dir) {
    if (!addComplete) {
        cout << "[NodeInstance::addExit] you add an exit to a completed node!" << endl;
    }
    dir = checkDir(dir);
    exits.emplace_back(posx, posy, dir);
    exitNums ++;
}

void NodeInstance::finishAdding() {
    sort(exits.begin(), exits.end());
    exitNums = static_cast<uint8_t>(exits.size());
    addComplete = true;
}

void NodeInstance::changeExtraMsgTo(const string &msg) {
    extraMsg = msg;
}

void NodeInstance::addExtreaMsg(const string &msg) {
    extraMsg += msg;
}

inline const string &NodeInstance::getExtraMsg() {
    return extraMsg;
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

inline void NodeInstance::addUseage(MapCandidate *usedMap, TopoNode *usedAt) {
    nodeUseages.insert({usedMap, usedAt});
}

inline void NodeInstance::removeUseage(MapCandidate *map2unbind) {
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
        cout << "plz call finishAdding" << endl;
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

    /**check the pos and dir of all exits*/
    auto lExit = lExits.cbegin();
    auto rExit = rExits.cbegin();
    for (int i = 0; i < exits.size(); i++) {
        double dirDif = abs( rExit->Dir() - lExit->Dir());
        if (dirDif > dirError()) {
            return false;
        }

        //TODO 改成二范数?
        double posDif = abs(rExit->getPosX() - lExit->getPosX()) + abs(rExit->getPosY() - lExit->getPosY());
        if (posDif > posError()) {
            return false;
        }
    }
    return true;
}

