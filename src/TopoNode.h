//
// Created by stumbo on 18-5-10.
//

#ifndef TOPOLOGY_MAP_TOPONODE_H
#define TOPOLOGY_MAP_TOPONODE_H

#include <vector>
#include <set>
#include <string>
#include <iostream>

#include "NodeExit.h"

class MapCandidate;

using namespace std;

/**
 * 拓扑节点类,仅保存结构,连接情况由地图类MapCandidate完成
 */
class TopoNode {
public:
    void addExit(double posx, double posy, double dir);
    void finishAdding();
    void changeExtraMsgTo(const string &msg);
    void addExtreaMsg(const string &msg);
    const string& getExtraMsg();
    unsigned long exitNums();
    const bool isAddComplete() const {
        return addComplete;
    }

    const vector<NodeExit> &getExits() const {
        return exits;
    }

    const set<shared_ptr<MapCandidate>> &getMapUsed() const {
        return mapUsed;
    }

    bool operator== (const TopoNode&) const;

private:
    /**
     * exits是否添加完成,exits应当是一次性添加后排序的
     */
    bool addComplete = false;
    vector<NodeExit> exits;
    string extraMsg;
    double & checkDir(double & d);
    set<shared_ptr<MapCandidate>> mapUsed;

    /**
     * @return 出口朝向的误差容忍度
     */
    static double dirError() {
        return 30.0;
    }

    /**
     * @return 出口判定点位置的误差荣热度
     */
    static double posError() {
        return 0.5;
    }
};


#endif //TOPOLOGY_MAP_TOPONODE_H
