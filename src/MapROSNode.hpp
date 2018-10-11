//
// Created by stumbo on 18-7-22.
//

#ifndef TOPOLOGY_MAP_MAPNODE_HPP
#define TOPOLOGY_MAP_MAPNODE_HPP

#include <iostream>
#include <fstream>

#include "topo/Topo.h"
#include "ros/ros.h"

#include <topology_map/NewNodeMsg.h>
#include <topology_map/SaveMap.h>
#include <std_msgs/UInt8.h>

using std::cout;
using std::endl;

class MapROSNode {
public:
    MapROSNode();

    explicit MapROSNode(const std::string &fileName);

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

    void rosNodeInit();

    void cbNewNode(const topology_map::NewNodeMsg &);
    void cbThroughGate(std_msgs::UInt8);
    bool srvSaveMap(topology_map::SaveMap::Request &, topology_map::SaveMap::Response &);
};

MapROSNode::MapROSNode() {
    rosNodeInit();
}

/**
 * start from a exist Map
 * @param fileName
 */
MapROSNode::MapROSNode(const std::string &fileName) {
    mapGroup.reloadFromFile(fileName);
    rosNodeInit();
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
    topoFile.outputMap(mapGroup);
    res.fileName = mapGroup.getMapName();
    return true;
}

void MapROSNode::rosNodeInit() {
    //TODO using client rather than msg?
    sub_NewNodeInfo = n.subscribe(TOPO_STD_TOPIC_NAME_NODEINFO, 5,
                                  &MapROSNode::cbNewNode, this);
    sub_GateMovement = n.subscribe(TOPO_STD_TOPIC_NAME_GATEMOVE, 5,
                                   &MapROSNode::cbThroughGate, this);
    srv_SaveMap = n.advertiseService(TOPO_STD_SERVICE_NAME_SAVEMAP, &MapROSNode::srvSaveMap, this);

}

#endif //TOPOLOGY_MAP_MAPNODE_HPP
