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
#include <topology_map/LeaveNode.h>
#include <topology_map/SaveMap.h>
#include <topology_map/GetMaps.h>
#include <topology_map/PathPlanning.h>
#include <topology_map/NextStepOfPath.h>
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
        srv_getMaps.shutdown();
        srv_PathPlan.shutdown();
        srv_AskedNextPathStep.shutdown();
    }


private:
    MapArranger mapGroup;
    ros::Subscriber sub_NewNodeInfo;
    ros::Subscriber sub_GateMovement;
    ros::ServiceServer srv_SaveMap;
    ros::ServiceServer srv_getMaps;
    ros::ServiceServer srv_PathPlan;
    ros::ServiceServer srv_AskedNextPathStep;
    ros::NodeHandle n;

    void rosNodeInit();

    void cbNewNode(const topology_map::NewNodeMsg &);
    void cbThroughGate(const topology_map::LeaveNode &);
    bool srvSaveMap(topology_map::SaveMap::Request &, topology_map::SaveMap::Response &);
    bool srvGetMap(topology_map::GetMaps::Request &, topology_map::GetMaps::Response &);
    bool srvPathPlanning(topology_map::PathPlanning::Request &, 
                         topology_map::PathPlanning::Response &);
    bool srvAskedPathStep(topology_map::NextStepOfPath::Request &,
                          topology_map::NextStepOfPath::Response &);
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
    ROS_INFO("START_NEW_NODE");
    auto nodeInstance = new NodeInstance();
    for (int i = 0; i < msgP.exitNum; i++) {
        nodeInstance->addExit(msgP.midPosXs[i], msgP.midPosYs[i], msgP.outDirs[i]);
    }
    nodeInstance->completeAdding();
    mapGroup.arriveInstance(nodeInstance, static_cast<gateId>(msgP.arriveAt), msgP.odomX,
                            msgP.odomY, msgP.odomYaw);
    ROS_INFO("END_NEW_NODE");

    auto k = mapGroup.getNodeCollection().experienceSize();
    auto sum = mapGroup.getMapCollection().calSumOfConfidence();
    auto & sorted = mapGroup.getMapCollection().getOrderedMaps();
    cout << "rec new node complete, current candidates: " << mapGroup.getMapNumbers() << endl;
    if (sorted.size() > 1) {
        cout << "\n best confidence: " << sorted[0]->getConfidence(k) / sum
             << "second best:" << sorted[1]->getConfidence(k) / sum << endl;
    }
//    for (auto & map : mapGroup.getMapCollection().getMaps()) {
//        cout << map->getConfidencePURE() << endl;
//    }
}

void MapROSNode::cbThroughGate(const topology_map::LeaveNode & leaveGate) {
    mapGroup.moveThroughGate(static_cast<gateId>(leaveGate.leaveGate));
    if (mapGroup.experienceNum() == 6) {
        cout << mapGroup.getMapNumbers() << endl;
    }
    cout << "rec new edge complete" << endl;
}

bool MapROSNode::srvSaveMap(topology_map::SaveMap::Request &req,
                         topology_map::SaveMap::Response &res) {
    TopoFile topoFile(mapGroup.getMapName());
    topoFile.open();
    topoFile.setSpliter("\t");
    mapGroup.sortByConfidence();
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
    srv_SaveMap = n.advertiseService(TOPO_STD_SERVICE_NAME_SAVEMAP,
            &MapROSNode::srvSaveMap, this);
    srv_getMaps = n.advertiseService(TOPO_STD_SERVICE_NAME_GETMAPS,
            &MapROSNode::srvGetMap, this);
    srv_PathPlan = n.advertiseService(TOPO_STD_SERVICE_NAME_PATHPLANNING, 
            &MapROSNode::srvPathPlanning, this);
    srv_AskedNextPathStep = n.advertiseService(TOPO_STD_SERVICE_NAME_ASKINGNEXTSTEP,
            &MapROSNode::srvAskedPathStep, this);

}

bool MapROSNode::srvGetMap(topology_map::GetMaps::Request & req,
                           topology_map::GetMaps::Response & res) {
    auto askMaps = req.requiredMaps;
    mapGroup.sortByConfidence(askMaps);
    auto str = mapGroup.toString(askMaps);

    ROS_INFO_STREAM("[srvGetMap] requset of map count "
                            << askMaps <<" is sent\nsize:" << str.size());

    res.mapJS = std::move(str);

    return true;
}

bool MapROSNode::srvPathPlanning(topology_map::PathPlanning::Request& req,
                                 topology_map::PathPlanning::Response& res)
{
    auto & resVec = res.pathInsSerialN;
    mapGroup.getMapCollection().sortByConfidence(req.rankOfPlanningMap);
    MapCandidate* map = mapGroup.getMapCollection().getOrderedMaps()[req.rankOfPlanningMap];
    TopoPath& topoPath = mapGroup.getMapCollection().getTopoPath();
    NodeInstance* beginIns =
            mapGroup.getNodeCollection().getExperiences()[req.beginInsSerialN];
    if (beginIns != map->getCurrentNode()->getInsCorrespond()) {
        cerr << __FILE__ << ":" << __LINE__ <<
                "[WARNING] the path plan request is out of date!" << endl;
    }
    NodeInstance* goalIns =
            mapGroup.getNodeCollection().getExperiences()[req.goalInsSerialN];
    bool success = topoPath.findPath(map, beginIns, goalIns);
    if (success) {
        auto& steps = topoPath.getPath();
        resVec.reserve(steps.size() + 1);
        for (auto & step : steps) {
            resVec.emplace_back(step.beginNode->getInsCorrespond()->getSerialNumber());
        }
        resVec.emplace_back(req.goalInsSerialN);
        return true;
    } else {
        return false;
    }
}

bool MapROSNode::srvAskedPathStep(topology_map::NextStepOfPath::Request& req,
                                  topology_map::NextStepOfPath::Response& res)
{
    TopoPath & topoPath = mapGroup.getMapCollection().getTopoPath();
    if (topoPath.hasSteps2Go()) {
        auto & path = topoPath.getPath();
        auto progress = topoPath.getCurrentProgress();
        auto & step = path[progress];
        TopoNode* currentNode = step.beginNode;
        res.gateId = currentNode->gateIdOfTheTopoEdge(step.stepEdge);
        res.nRestEdge = path.size() - progress;
        auto& exitInstance = currentNode->getInsCorrespond()->getExits()[res.gateId];
        res.midPosX = exitInstance.getPosX();
        res.midPosY = exitInstance.getPosY();
        res.outDir = exitInstance.getOutDir();
    } else {
        if (!topoPath.isValid()) {
            res.nRestEdge = -1;
        }
        else if (topoPath.isFinished()) {
            res.nRestEdge = 0;
        }
        else if (!topoPath.isFollowingPath()) {
            res.nRestEdge = -2;
        }
    }
    return true;
}

#endif //TOPOLOGY_MAP_MAPNODE_HPP
