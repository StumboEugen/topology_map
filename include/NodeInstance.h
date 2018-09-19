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

#include <topology_map/NewNodeMsg.h>

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
    NodeInstance();
    void addExit(double posx, double posy, double dir);
    void completeAdding();
    void addUseage(MapCandidate *usedMap, TopoNode *usedAt);
    void removeUseage(MapCandidate *map2unbind);
    bool alike (const NodeInstance&) const;

    //TODO slove the problem of switch between gateID and ExitPos

    topology_map::NewNodeMsgPtr encode2ROSmsg(unsigned char arriveAt, float odomX, float odomY);

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

    static double exitRadTollerance() {
        return 0.3;
    }

    void changeExtraMsgTo(const string &msg) {
        extraMsg = msg;
    }

    void addExtreaMsg(const string &msg) {
        extraMsg += msg;
    }

    inline const string &getExtraMsg() {
        return extraMsg;
    }

    size_t getSerialNumber() const {
        return serialNumber;
    }

    void setSerialNumber(size_t serialNumber) {
        NodeInstance::serialNumber = serialNumber;
    }


    static size_t serialCount;

private:
    /** exits是否添加完成,exits应当是一次性添加后排序的 */
    bool addComplete = false;
    vector<ExitInstance> exits;
    uint8_t exitNums = 0;
    string extraMsg;
    const double & checkDir(double & d);
    map<MapCandidate*, TopoNode*> nodeUseages;
    size_t serialNumber;
};


#endif //TOPOLOGY_MAP_NODEINSTANCE_H
