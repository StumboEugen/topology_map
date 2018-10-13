//
// Created by stumbo on 18-5-10.
//

#include "MapROSNode.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <list>

#include <topology_map/NewNodeMsg.h>

#include "topo/Topo.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "TopoLayer");

    ros::NodeHandle n;
    
    MapROSNode mapNode;    //callbacks are in this class

    ros::spin();

    /**
     * without these lines, debug will fail
     */
    auto xxxxx = new NodeInstance();
    std::list<MapCandidate*> mm;
    auto forDebug = new MapCandidate(xxxxx);
    mm.push_back(forDebug);

    MapArranger mapGroup;
//    auto instance0 = new NodeInstance();
//    instance0->addExit(1, 0, 0);
//    instance0->addExit(0, -1, 90);
//    instance0->completeAdding();
//    mapGroup.arriveInstance(instance0, 0, 0, 0);
//    mapGroup.moveThroughGate(0);
//
//    auto instance1 = new NodeInstance();
//    instance1->addExit(-1, 0, 0);
//    instance1->addExit(0, -1, 90);
//    instance1->completeAdding();
//    mapGroup.arriveInstance(instance1, 0, 10, 0);
//    mapGroup.moveThroughGate(1);
//
//    auto instance2 = new NodeInstance();
//    instance2->addExit(-1, 0, 0);
//    instance2->addExit(0, 1, 180);
//    instance2->completeAdding();
//    mapGroup.arriveInstance(instance2, 1, 0, -10);
//    mapGroup.moveThroughGate(0);
//
//    auto instance3 = new NodeInstance();
//    instance3->addExit(1, 0, 0);
//    instance3->addExit(0, 1, 180);
//    instance3->completeAdding();
//    mapGroup.arriveInstance(instance3, 1, -10, 0);
//    mapGroup.moveThroughGate(0);
//
//    auto instance4 = new NodeInstance();
//    instance4->addExit(1, 0, 0);
//    instance4->addExit(0, -1, 90);
//    instance4->completeAdding();
//    mapGroup.arriveInstance(instance4, 1, 0, 10);
//    mapGroup.moveThroughGate(0);
//
//    auto instance5 = new NodeInstance();
//    instance5->addExit(-1, 0, 0);
//    instance5->addExit(0, -1, 90);
//    //instance5->addExit(1, 1, 90);
//    instance5->completeAdding();
//    mapGroup.arriveInstance(instance5, 0, 10, 0);
//    mapGroup.moveThroughGate(1);


    return 0;
}