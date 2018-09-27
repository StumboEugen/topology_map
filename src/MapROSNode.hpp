//
// Created by stumbo on 18-7-22.
//

#ifndef TOPOLOGY_MAP_MAPNODE_HPP
#define TOPOLOGY_MAP_MAPNODE_HPP

#include <iostream>
#include <fstream>

#include "topo/TopoMap.h"
#include "ros/ros.h"

#include <topology_map/NewNodeMsg.h>
#include <topology_map/SaveMap.h>
#include <std_msgs/UInt8.h>

using std::cout;
using std::endl;

class MapROSNode {
public:
    MapROSNode();

    ~MapROSNode() {
        sub_NewNodeInfo.shutdown();
        sub_GateMovement.shutdown();
        srv_SaveMap.shutdown();
    }


private:
    MapArranger mapGroup;
    ros::Subscriber sub_NewNodeInfo;
    ros::Subscriber sub_GateMovement;
    ros::ServiceServer srv_SaveMap;
    ros::NodeHandle n;

    void cbNewNode(const topology_map::NewNodeMsg &);
    void cbThroughGate(std_msgs::UInt8);
    bool srvSaveMap(topology_map::SaveMap::Request &, topology_map::SaveMap::Response &);
};

MapROSNode::MapROSNode() {
    //TODO using client rather than msg?
    sub_NewNodeInfo = n.subscribe(TOPO_STD_TOPIC_NAME_NODEINFO, 1,
                                  &MapROSNode::cbNewNode, this);
    sub_GateMovement = n.subscribe(TOPO_STD_TOPIC_NAME_GATEMOVE, 1,
                                   &MapROSNode::cbThroughGate, this);
    srv_SaveMap = n.advertiseService(TOPO_STD_SERVICE_NAME_SAVEMAP, &MapROSNode::srvSaveMap, this);
}

void MapROSNode::cbNewNode(const topology_map::NewNodeMsg &msgP) {
    auto nodeInstance = new NodeInstance();
    for (int i = 0; i < msgP.exitNum; i++) {
        nodeInstance->addExit(msgP.midPosXs[i], msgP.midPosYs[i], msgP.outDirs[i]);
    }
    nodeInstance->completeAdding();
    mapGroup.arriveInstance(nodeInstance, static_cast<gateId>(msgP.arriveAt),
                            msgP.odomX, msgP.odomY);
    cout << "rec new node" << endl;
}

void MapROSNode::cbThroughGate(std_msgs::UInt8 leaveGate) {
    mapGroup.moveThroughGate(leaveGate.data);
    if (mapGroup.experienceNum() == 6) {
        cout << mapGroup.getMapNumbers() << endl;
    }
    cout << "rec new edge" << endl;
}

bool MapROSNode::srvSaveMap(topology_map::SaveMap::Request &req,
                         topology_map::SaveMap::Response &res) {
    TopoFile topoFile(mapGroup.getMapName());
    topoFile.open();
    topoFile.writeMap(mapGroup);
    res.fileName = mapGroup.getMapName();
    return true;
}

#endif //TOPOLOGY_MAP_MAPNODE_HPP