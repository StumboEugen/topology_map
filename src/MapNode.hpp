//
// Created by stumbo on 18-7-22.
//

#ifndef TOPOLOGY_MAP_MAPNODE_HPP
#define TOPOLOGY_MAP_MAPNODE_HPP

#include <iostream>

#include "TopoMap.h"
#include "ros/ros.h"

#include <topology_map/NewNodeMsg.h>
#include <std_msgs/UInt8.h>

using std::cout;
using std::endl;

class MapNode {
public:
    MapNode();

    ~MapNode() {
        newNodeInfo_sub.shutdown();
        gateMovement_sub.shutdown();
    }


private:
    MapArranger mapGroup;
    ros::Subscriber newNodeInfo_sub;
    ros::Subscriber gateMovement_sub;
    ros::NodeHandle n;

    void callbackNewNode(const topology_map::NewNodeMsg &);
    void callbackThroughGate(std_msgs::UInt8);
};

MapNode::MapNode() {
    newNodeInfo_sub = n.subscribe(TOPO_STD_TOPIC_NAME_NODEINFO, 1,
            &MapNode::callbackNewNode, this);
    gateMovement_sub = n.subscribe(TOPO_STD_TOPIC_NAME_GATEMOVE, 1,
            &MapNode::callbackThroughGate, this);
}

void MapNode::callbackNewNode(const topology_map::NewNodeMsg &msgP) {
    auto nodeInstance = new NodeInstance();
    for (int i = 0; i < msgP.exitNum; i++) {
        nodeInstance->addExit(msgP.midPosXs[i], msgP.midPosYs[i], msgP.outDirs[i]);
    }
    nodeInstance->completeAdding();
    mapGroup.arriveInstance(nodeInstance, static_cast<gateId>(msgP.arriveAt),
                            msgP.odomX, msgP.odomY);
}

void MapNode::callbackThroughGate(const std_msgs::UInt8 leaveGate) {
    mapGroup.moveThroughGate(leaveGate.data);
    if (mapGroup.experienceNum() == 6) {
        cout << mapGroup.getMapNumbers() << endl;
    }
}

#endif //TOPOLOGY_MAP_MAPNODE_HPP
