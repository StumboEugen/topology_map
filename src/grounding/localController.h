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

const static float curiseHeight = 0.77;

/// 60 degree cam
const static float m2pxWithUnitZ = 600;

const static float Z_INC_MAX = 0.3;
const static float Z_INC_MIN = 0.1;
const static float Z_TOLLERANCE = 0.1;
const static float Z_SAFEDIS = 0.3;

const static float XY_INC_MAX = 0.3;
const static float XY_INC_MIN = 0.08;
const static float XY_TOLLERANCE = 0.1;
const static float XY_SAFEDIS = 0.2;

ros::Publisher pub_spPose;
ros::Publisher pub_takeOff;
ros::Publisher pub_newNode;

TopoLocalControllerMode mode = MODE_AIMMING_AT_NODE;

int automony_status = 0;

/// ENU N is 0, -pi ~ pi clockwise
px4_autonomy::Position curPose;

/// ENU N is 0, -pi ~ pi clockwise
vec3f_t curSP;
px4_autonomy::Position posCmd;

/// image coor, right x, down y, rh th is in ENU
topology_map::ImageExract imageInfo;

float lastNodeX = 0.0f;
float lastNodeY = 0.0f;
bool aimComplete = false;

float curMovingDIR = pi;
float midInImgx;
float midInImgy;
float m2px;

void cb_px4Pose(const px4_autonomy::Position &);
void cb_status(const std_msgs::UInt8 &);
void cb_image(const topology_map::ImageExract &);
void cb_gateMove(const topology_map::LeaveNode &);

void findTheLineAndGiveSP();

void move2Z(float targetZ) {

    ros::Rate rate(RFRATE);

    curSP.x = curPose.x;
    curSP.y = curPose.y;

    while (fabsf(curiseHeight - curPose.z) > Z_TOLLERANCE) {
        curSP.z = curPose.z + slopeCal(curiseHeight - curPose.z,
                                       Z_TOLLERANCE, Z_SAFEDIS,
                                       Z_INC_MIN, Z_INC_MAX);
        curSP.loadXYZ2POS(posCmd);
        pub_spPose.publish(posCmd);
        rate.sleep();
        ros::spinOnce();
    }
    curSP.z = targetZ;
    curSP.loadXYZ2POS(posCmd);
    pub_spPose.publish(posCmd);
}

void stayAtSP(double ms = 0.0) {
    curSP.loadXYZ2POS(posCmd);
    pub_spPose.publish(posCmd);
    ROSSLEEP(ms);
    ros::spinOnce();
}

double fixRad2nppi(double target) {
    if (target > pi) {
        target -= piTwo;
    }
    else if (target < -pi) {
        target += piTwo;
    }
    return target;
}

#endif //TOPOLOGY_MAP_LOCALCONTROLLER_H
