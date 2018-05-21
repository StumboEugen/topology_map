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
        d =- 360.0 * (int)(d/360.0);
    }
    return d;
}
/**
 * compare two nodes if it IS ALIKE
 * @param rnode
 * @return is it alike
 */
bool NodeInstance::operator==(const NodeInstance & rnode) const {
    const NodeInstance & lnode = *this;

    //完成添加后才可以比较
    if (!this->addComplete || !rnode.isAddComplete()) {
        cout << "[WARNING] compare nodes before add Complete!!" << endl;
        cout << "plz call finishAdding" << endl;
    }

    //特殊信息是否相同
    if (lnode.extraMsg != rnode.extraMsg) {
        return false;
    }

    //出口数量是否相同
    auto & lv = lnode.exits;
    auto & rv = rnode.exits;
    if (lv.size() != rv.size()) {
        return false;
    }

    //每个出口逐一对照位置和出口方向
    auto li = lv.cbegin();
    auto ri = rv.cbegin();
    for (int i = 0; i < exits.size(); i++) {
        double dirDif = abs( ri->Dir() - li->Dir());
        if (dirDif > dirError()) {
            return false;
        }

        //TODO 改成二范数?
        double posDif = abs(ri->getPosX() - li->getPosX()) + abs(ri->getPosY() - li->getPosY());
        if (posDif > posError()) {
            return false;
        }
    }
    return true;
}

