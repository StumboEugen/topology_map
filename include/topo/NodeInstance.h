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

#include "TopoTools.h"
#include "ExitInstance.h"

using std::map;
using std::vector;
using std::string;

/**
 * represent a real node, stored in the NodeCollection
 * the connect information is stored in map candidate
 */
class NodeInstance {
public:
    explicit NodeInstance(bool registerSerial = true);

    NodeInstance(const NodeInstance &, bool registerSerial = true);
    
    /**
     * add an exit to this instance (ENU)
     * @param posx the posx relative to the middle of the instance
     * @param posy the posy relative to the middle of the instance
     * @param dir the outward of the exit 0-360
     * @example instance like 'G', exit is like : (1,1,120) and (0,0,270)
     */
    void addExit(double posx, double posy, double dir);
    
    /**
     * after add, sort the exits
     */
    void completeAdding();
    
    /**
     * add the map useage to this node instance
     * @param usedMap 
     * @param usedAt 
     */
    void addUseage(MapCandidate *usedMap, TopoNode *usedAt);
    
    /**
     * remove the map useage form the record
     * @param map2unbind 
     */
    void removeUseage(MapCandidate *map2unbind);

    /**
     * compare two nodes if it IS ALIKE
     * @param rnode
     * @return -1: unlike  other: the serial ID difference, 1: the serial of this need to +1
     */
    int alike(const NodeInstance &) const;

    // TODO slove the problem of switch between gateID and ExitPos

    topology_map::NewNodeMsgPtr
    encode2ROSmsg(unsigned char arriveAt, float odomX, float odomY, float yaw);

    topology_map::NewNodeMsgPtr
    encode2ROSmsg(unsigned char arriveAt, const array<double, 3> & odomMsg);
    // TODO split this(ROS related) out of the lib

    const uint8_t sizeOfExits() const {
        return exitNums;
    }

    const bool isAddComplete() const {
        return addComplete;
    }

    /**
     * get the cloest exit of the dir
     * @param midDir in rad
     * @return the closest exit id
     */
    gateId getMidDirClosestExit(double midDir); //TODO

    inline const map<MapCandidate *, TopoNode *> &getNodeUseages() const {
        return nodeUseages;
    }

    const vector<ExitInstance> & getExits() const {
        return exits;
    }

    bool haveNoUseages() {
        return nodeUseages.empty();
    }

    size_t getSerialNumber() const {
        return serialNumber;
    }

    void setSerialNumber(size_t serialNumber) {
        NodeInstance::serialNumber = serialNumber;
    }

    // accumulate the serial Number
    static size_t serialCount;

    JSobj toJS() const;

    /**
     * set the pos in global,(from the origin) and the distance moved
     */
    void setGlobalPos(double x, double y, double movedDis);

    double getGlobalX() const {
        return globalX;
    }

    double getGlobalY() const {
        return globalY;
    }

    double getTravelDis() const {
        return travelDis;
    }

private:
    /**
     * check & adjust the dir to 0-360°
     * @param d
     * @return the adjusted direction
     */
    const double & checkDir(double & d);

    static double exitRadTollerance() {
        return 0.5;
    }

    void setExtraMsg(const string &msg) {
        extraMsg = msg;
    }

    void addExtreaMsg(const string &msg) {
        extraMsg += msg;
    }

    inline const string &getExtraMsg() {
        return extraMsg;
    }

    // determine if the instance is add complete
    bool addComplete = false;

    //container to store the exit infos
    vector<ExitInstance> exits;
    uint8_t exitNums = 0;

    // extra semantic info
    string extraMsg;

    //the useage of the nodes, in which map at which node
    map<MapCandidate*, TopoNode*> nodeUseages;

    //serial number record the input sequence & pointer like number
    size_t serialNumber;    //TODO 0 start or 1 start?

    double globalX;
    double globalY;
    double travelDis;

};


#endif //TOPOLOGY_MAP_NODEINSTANCE_H
