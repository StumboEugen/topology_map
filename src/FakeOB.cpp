#include "ros/ros.h"

#include "TopoMap.h"

#include <topology_map/NewNodeMsg.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher nodeInfo_pub = n.advertise<topology_map::NewNodeMsg>(TOPO_STD_TOPIC_NAME_NODEINFO, 0);
    ros::Publisher gateMove_pub = n.advertise<std_msgs::UInt8>(TOPO_STD_TOPIC_NAME_GATEMOVE, 0);

    sleep(1);

    ros::Rate loop_rate(10);

    MapArranger mapGroup;

    auto instance0 = new NodeInstance();    //TODO use shared_ptr or delete the pointer
    instance0->addExit(1, 0, 0);
    instance0->addExit(0, -1, 90);
    instance0->completeAdding();
    auto nodeMsg = instance0->encode2ROSmsg(0, 0, 0);
    std_msgs::UInt8 tempGate;
    tempGate.data = 0;
    nodeInfo_pub.publish(nodeMsg);
    loop_rate.sleep();
    gateMove_pub.publish(tempGate);

    auto instance1 = new NodeInstance();
    instance1->addExit(-1, 0, 0);
    instance1->addExit(0, -1, 90);
    instance1->completeAdding();
    nodeMsg = instance1->encode2ROSmsg(0, 10, 0);
    tempGate.data = 1;
    nodeInfo_pub.publish(nodeMsg);
    loop_rate.sleep();
    gateMove_pub.publish(tempGate);

    auto instance2 = new NodeInstance();
    instance2->addExit(-1, 0, 0);
    instance2->addExit(0, 1, 180);
    instance2->completeAdding();
    nodeMsg = instance2->encode2ROSmsg(1, 0, -10);
    tempGate.data = 0;
    nodeInfo_pub.publish(nodeMsg);
    loop_rate.sleep();
    gateMove_pub.publish(tempGate);

    auto instance3 = new NodeInstance();
    instance3->addExit(1, 0, 0);
    instance3->addExit(0, 1, 180);
    instance3->completeAdding();
    nodeMsg = instance3->encode2ROSmsg(1, -10, 0);
    tempGate.data = 0;
    nodeInfo_pub.publish(nodeMsg);
    loop_rate.sleep();
    gateMove_pub.publish(tempGate);

    auto instance4 = new NodeInstance();
    instance4->addExit(1, 0, 0);
    instance4->addExit(0, -1, 90);
    instance4->completeAdding();
    nodeMsg = instance4->encode2ROSmsg(1, 0, 10);
    tempGate.data = 0;
    nodeInfo_pub.publish(nodeMsg);
    loop_rate.sleep();
    gateMove_pub.publish(tempGate);

    auto instance5 = new NodeInstance();
    instance5->addExit(-1, 0, 0);
    instance5->addExit(0, -1, 90);
    //instance5->addExit(1, 1, 90);
    instance5->completeAdding();
    nodeMsg = instance5->encode2ROSmsg(0, 10, 0);
    tempGate.data = 1;
    nodeInfo_pub.publish(nodeMsg);
    loop_rate.sleep();
    gateMove_pub.publish(tempGate);

    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        gateMove_pub.publish(tempGate);
        nodeInfo_pub.publish(nodeMsg);

        loop_rate.sleep();
        ++count;
    }

    return 0;
}