//
// Created by stumbo on 18-5-10.
//


#ifndef TOPOLOGY_MAP_NODEINSTANCE_H
#define TOPOLOGY_MAP_NODEINSTANCE_H

#include <vector>
#include <set>
#include <map>
#include <string>
#include <iostream>

using std::map;
using std::vector;
using std::string;

class ExitInstance;
class MapCandidate;
class TopoNode;

/**
 * represent a real node, stored in the NodeCollection
 * the connect information is stored in map candidate
 */
class NodeInstance {
public:
    void addExit(double posx, double posy, double dir);
    void finishAdding();
    void changeExtraMsgTo(const string &msg);
    void addExtreaMsg(const string &msg);
    void addUseage(MapCandidate *usedMap, TopoNode *usedAt);
    void removeUseage(MapCandidate *map2unbind);
    const string& getExtraMsg();
    bool alike (const NodeInstance&) const;

    const uint8_t sizeOfExits() const {
        return exitNums;
    }

    const bool isAddComplete() const {
        return addComplete;
    }

    inline const map<MapCandidate *, TopoNode *> &getNodeUseages() const {
        return nodeUseages;
    }

    bool haveNoUseages() {
        return nodeUseages.empty();
    }

private:
    /** exits是否添加完成,exits应当是一次性添加后排序的 */
    bool addComplete = false;
    vector<ExitInstance> exits;
    uint8_t exitNums;
    string extraMsg;
    const double & checkDir(double & d);
    map<MapCandidate*, TopoNode*> nodeUseages;

    /**
     * @return 出口朝向的误差容忍度
     */
    static double dirError() {
        return 30.0;
    }

    /**
     * @return 出口判定点位置的误差容忍度
     */
    static double posError() {
        return 0.5;
    }
};


#endif //TOPOLOGY_MAP_NODEINSTANCE_H
