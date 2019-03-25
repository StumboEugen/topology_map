//
// Created by stumbo on 19-3-25.
//

#include "ros/ros.h"

#include "topo/TopoTools.h"

#include "localController.h"

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "localController");
    ros::NodeHandle n;

    ros::Subscriber sub_px4pose = n.subscribe("/px4/pose", 1, cb_px4Pose);
    ros::Subscriber sub_status = n.subscribe("/px4/status", 1, cb_status);
    ros::Subscriber sub_image = n.subscribe(TOPO_STD_TOPIC_NAME_CVINFO, 1, cb_image);
    ros::Subscriber sub_gateMove = n.subscribe(TOPO_STD_TOPIC_NAME_GATEMOVE, 1, cb_gateMove);

    pub_spPose = n.advertise<px4_autonomy::Position>("/px4/cmd_pose", 1);
    pub_takeOff = n.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeOff", 1);
    pub_newNode = n.advertise<topology_map::NewNodeMsg>(TOPO_STD_TOPIC_NAME_NODEINFO, 1);

    /// take off
    while (automony_status != 1) {
        ROSSLEEP(1);
        ROS_INFO_THROTTLE(2, "[TAKE OFF] waitting for OFFBOARD");
        ros::spinOnce();
    }

    px4_autonomy::Takeoff cmd_tf;
    cmd_tf.take_off = 1;
    cmd_tf.header.stamp = ros::Time::now();
    pub_takeOff.publish(cmd_tf);

    while (automony_status != 5) {
        ROSSLEEP(0.2);
        ROS_INFO_THROTTLE(2, "[TAKE OFF] taking off...");
        ros::spinOnce();
    }

    move2Z(curiseHeight);

    ros::Rate rate(RFRATE);

    while (ros::ok()) {

        switch (mode) {

            case MODE_ON_EDGE:

                if (imageInfo.nodePosX != -1) {
                    // TODO CHECK IF THE CROSS IS AT THE MOVING DIR
                }

                //TODO give the sp to follow line

                break;
            case MODE_ARRIVING_NODE:

                //TODO give the sp to follow line

                if (!imageInfo.exitDirs.empty()) {
                    mode = MODE_AIMMING_AT_NODE;
                }

                break;
            case MODE_AIMMING_AT_NODE:
                if (imageInfo.exitDirs.empty()) {
                    static int errorCount = 0;
                    errorCount++;
                    if (errorCount >= 3) {
                        cerr << "NO CROSS NODE DETECTED AT AIMMING MODE! throw!" << endl;
                        exit(0);
                    }
                }

                //TODO AIMMING WORK

                //TODO if aimming good, send the topo structor

                break;
            case MODE_LEAVING_NODE:

                //TODO give the sp to follow line

                if (imageInfo.nodePosX != -1) {
                    mode = MODE_ON_EDGE;
                }

                break;
        }

        rate.sleep();
    }
}

void cb_status(const std_msgs::UInt8 & msg) {
    automony_status = msg.data;
}

void cb_px4Pose(const px4_autonomy::Position & msg) {
    curPose = msg;
}

void cb_image(const topology_map::ImageExract & msg) {
    imageInfo = msg;
}

void cb_gateMove(const topology_map::LeaveNode &msg) {
    if (mode == MODE_AIMMING_AT_NODE) {
        mode = MODE_LEAVING_NODE;
        curMovingDIR = msg.leaveDir;
    }
}
