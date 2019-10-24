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
 * represent a real node and include detail information, stored in the NodeCollection.
 * the connect information of a map is stored in MapCandidate
 */
class NodeInstance {
public:

    // purest constructor of a empty NodeInstance
    explicit NodeInstance(bool registerSerial = true);

    // copy constructor from another NodeInstance
    NodeInstance(const NodeInstance &, bool registerSerial = true);

    //add an exit to this instance (ENU)
    void addExit(double posx, double posy, double dir);
    
    // the final step of adding is sorting exitInstances
    void completeAdding();

    // add the map useage to this node instance
    void addUseage(MapCandidate *usedMap, TopoNode *usedAt);

    // remove the map useage form the record
    void removeUseage(MapCandidate *map2unbind);

    // compare another node if they are alike
    int alike(const NodeInstance &) const;

    // TODO maybe using pos rather than id is better
    // encode the instance to the rosMsg, which need 3 paras about the movement to the node
    topology_map::NewNodeMsgPtr
    encode2ROSmsg(unsigned char arriveAt, float odomX, float odomY, float yaw);

    // get the cloest exit of the dir
    gateId getMidDirClosestExit(double midDir);

    // convert NodeInstance to JSON structure
    JSobj toJS() const;

    //set the pos in global, (from the origin) and the distance moved
    void setGlobalPos(double x, double y, double movedDis);

    // find the closest exit of the given pos
    gateId figureOutWhichExitItis(double posx, double posy);

    const uint8_t sizeOfExits() const {
        return exitNums;
    }

    const bool isAddComplete() const {
        return addComplete;
    }

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

    double getGlobalX() const {
        return globalX;
    }

    double getGlobalY() const {
        return globalY;
    }

    double getTravelDis() const {
        return travelDis;
    }

private: // functions

    /// check & adjust the dir to 0-360°
    const double & checkDir(double & d);

    // the tollerance of the exit rad (not the midRad)
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

private: // member
    /**
     * container to store the exit infos.
     * after completeAdding() is called
     * they will be sorted according to the mid dir (ENU atan2)
     * @see ExitInstance::operator<()
     */
    vector<ExitInstance> exits;

    /**
     * each NodeInstance has a unique serial number, accumulated from this
     * @see serialNumber
     * @todo the static solution might be a problem, this should be arranged by NodeCollection
     */
    static size_t serialCount;

    /**
     * the unique serial number of this NodeInstance in one NodeCollection.
     * it is designed for JSON storage and communication
     * @see toJS
     * @see MapArranger::readFromJSON
     * @note in the real maps, the serial# starts from 0
     */
    size_t serialNumber;

    /// a safe flag to determine if the instance is added complete
    bool addComplete = false;

    /// count of the exitNums, the same as exits.size()
    uint8_t exitNums = 0;

    /// extra semantic info
    string extraMsg;

    /// the useage of the nodes, in which map at which node
    map<MapCandidate*, TopoNode*> nodeUseages;

    /// the odom pos X
    double globalX;

    /// the odom pos Y
    double globalY;

    /// the distance travelled, to calculate the odom error accumulation
    double travelDis;

};


#endif //TOPOLOGY_MAP_NODEINSTANCE_H
