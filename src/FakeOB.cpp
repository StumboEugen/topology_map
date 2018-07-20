#include "ros/ros.h"

#include "TopoMap.h"

#include <topology_map/NewNode.h>

//TODO finish the first communication via ROS, split this away from TopoMap
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<topology_map::NewNode>("nodeTraveller", 1);

    ros::Rate loop_rate(100);

    MapArranger mapGroup;

    auto instance0 = new NodeInstance();
    instance0->addExit(1, 0, 0);
    instance0->addExit(0, -1, 90);
    instance0->completeAdding();
    mapGroup.arriveInstance(instance0, 0, 0, 0);
    mapGroup.moveThroughGate(0);

    auto instance1 = new NodeInstance();
    instance1->addExit(-1, 0, 0);
    instance1->addExit(0, -1, 90);
    instance1->completeAdding();
    mapGroup.arriveInstance(instance1, 0, 10, 0);
    mapGroup.moveThroughGate(1);

    auto instance2 = new NodeInstance();
    instance2->addExit(-1, 0, 0);
    instance2->addExit(0, 1, 180);
    instance2->completeAdding();
    mapGroup.arriveInstance(instance2, 1, 0, -10);
    mapGroup.moveThroughGate(0);

    auto instance3 = new NodeInstance();
    instance3->addExit(1, 0, 0);
    instance3->addExit(0, 1, 180);
    instance3->completeAdding();
    mapGroup.arriveInstance(instance3, 1, -10, 0);
    mapGroup.moveThroughGate(0);

    auto instance4 = new NodeInstance();
    instance4->addExit(1, 0, 0);
    instance4->addExit(0, -1, 90);
    instance4->completeAdding();
    mapGroup.arriveInstance(instance4, 1, 0, 10);
    mapGroup.moveThroughGate(0);

    auto instance5 = new NodeInstance();
    instance5->addExit(-1, 0, 0);
    instance5->addExit(0, -1, 90);
    //instance5->addExit(1, 1, 90);
    instance5->completeAdding();
    mapGroup.arriveInstance(instance5, 0, 10, 0);
    mapGroup.moveThroughGate(1);

    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        topology_map::NewNode newNode;


        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(newNode);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}