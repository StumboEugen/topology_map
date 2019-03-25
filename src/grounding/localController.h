//
// Created by stumbo on 19-3-25.
//

#ifndef TOPOLOGY_MAP_LOCALCONTROLLER_H
#define TOPOLOGY_MAP_LOCALCONTROLLER_H

#include <topology_map/LeaveNode.h>
#include <topology_map/ImageExract.h>
#include <topology_map/NewNodeMsg.h>

#include <px4_autonomy/Takeoff.h>
#include <px4_autonomy/Position.h>

#include <std_msgs/UInt8.h>

#include "tools.h"

#include <cmath>

#define ROSSLEEP(time) ros::Duration(time).sleep()

enum TopoLocalControllerMode{
    MODE_ON_EDGE,
    MODE_ARRIVING_NODE,
    MODE_AIMMING_AT_NODE,
    MODE_LEAVING_NODE
};

const static float RFRATE = 10; // hz

const static float curiseHeight = 0.6;

const static float Z_INC_MAX = 0.3;
const static float Z_INC_MIN = 0.1;
const static float Z_TOLLERANCE = 0.1;
const static float Z_SAFEDIS = 0.3;

const static float XY_INC_MAX = 0.3;
const static float XY_INC_MIN = 0.1;
const static float XY_TOLLERANCE = 0.1;
const static float XY_SAFEDIS = 0.2;

ros::Publisher pub_spPose;
ros::Publisher pub_takeOff;
ros::Publisher pub_newNode;

TopoLocalControllerMode mode = MODE_AIMMING_AT_NODE;

int automony_status = 0;
px4_autonomy::Position curPose;
vec3f_t curSP;
topology_map::ImageExract imageInfo;

///in RAD, NED
float curMovingDIR;

void cb_px4Pose(const px4_autonomy::Position &);
void cb_status(const std_msgs::UInt8 &);
void cb_image(const topology_map::ImageExract &);
void cb_gateMove(const topology_map::LeaveNode &);

void move2Z(float targetZ) {

    ros::Rate rate(RFRATE);

    curSP.x = curPose.x;
    curSP.y = curPose.y;

    while (fabsf(curiseHeight - curPose.z) > Z_TOLLERANCE) {
        curSP.z = curPose.z + slopeCal(curiseHeight - curPose.z,
                                       Z_TOLLERANCE, Z_SAFEDIS,
                                       Z_INC_MIN, Z_INC_MAX);
        pub_spPose.publish(curSP.toPosCmd());
        rate.sleep();
        ros::spinOnce();
    }
    curSP.z = targetZ;
    pub_spPose.publish(curSP.toPosCmd());
}

void stayAtSP(double ms = 0.0) {
    pub_spPose.publish(curSP.toPosCmd());
    ROSSLEEP(ms);
    ros::spinOnce();
}

float fixRadFromImg(float radFromImage) {
    radFromImage += piHalf;
    if (radFromImage > piTwo) {
        radFromImage -= piTwo;
    }
    if (radFromImage < 0) {
        radFromImage += piTwo;
    }
    return radFromImage;
}

float fixRadFromImg(const vec3f_t & vecInImg) {

}

vec3f_t coorChangeFromImg(vec3f_t vec3f) {
    vec3f.y *= -1;
    return vec3f;
}

#endif //TOPOLOGY_MAP_LOCALCONTROLLER_H
