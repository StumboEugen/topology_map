//
// Created by stumbo on 18-5-10.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <MapArranger.h>

#include "TopoMap.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);



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
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}