//
// Created by stumbo on 19-3-25.
//

#include "ros/ros.h"

#include "topo/Topo.h"

#include "localController.h"

#include <iostream>

using namespace std;

/// in test mode:
/// HZ is 1,
/// do not take off
/// direct pass the aimming mode
bool testMode = true;

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
    if (!testMode) {
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

        posCmd.yaw = curPose.yaw;

        move2Z(curiseHeight);
    } else {
        m2px = m2pxWithUnitZ / 0.5f;
        midInImgx = 320;
        midInImgy = -240;
    }
        
    ros::Rate rate(RFRATE);

    while (ros::ok()) {

        ros::spinOnce();

        switch (mode) {

            case MODE_ON_EDGE:

                cout << "\nat mode: MODE_ON_EDGE" << endl;

                if (imageInfo.nodePosX != -1) {
                    float thOfNode = atan2f(
                            imageInfo.nodePosY - midInImgy,
                            imageInfo.nodePosX - midInImgx);
                    cout << "the node dir is " << thOfNode << endl;
                    const auto diffABS = fabsf(thOfNode - curMovingDIR);
                    if (diffABS < piHalf || diffABS - piTwo < piHalf) {
                        cerr << "find a node in the front! we are landing!!" << endl;
                        mode = MODE_ARRIVING_NODE;
                    }
                }

                findTheLineAndGiveSP();

                break;
            case MODE_ARRIVING_NODE:

                cout << "\nat mode: MODE_ARRIVING_NODE" << endl;

                findTheLineAndGiveSP();

                if (!imageInfo.exitDirs.empty()) {
                    mode = MODE_AIMMING_AT_NODE;
                }

                break;
            case MODE_AIMMING_AT_NODE: {

                cout << "\nat mode: MODE_AIMMING_AT_NODE" << endl;

                if (imageInfo.exitDirs.empty()) {

                    cerr << "NO CROSS NODE DETECTED AT AIMMING MODE!" << endl;

                    static int errorCount = 0;
                    errorCount++;
                    if (errorCount >= 3) {
                        ROS_FATAL_STREAM("NO CROSS NODE DETECTED AT AIMMING MODE FOR 3 times! "
                                         "throw!");
                        stayAtSP();
//                        exit(0);
                    }

                    break;
                }

                float aimErrx = imageInfo.nodePosX - midInImgx;
                float aimErry = imageInfo.nodePosY - midInImgy;
                float aimErrinPx = sqrtf(aimErrx * aimErrx + aimErry * aimErry);

                float aimErrinMeter = aimErrinPx / m2px;

                float aimErrTh = atan2f(aimErry, aimErrx) + curPose.yaw;

                /// TODO  check if the coor is good in meter
                float aimCorr = slopeCal(aimErrinMeter, 0.05, XY_SAFEDIS, 0.25, XY_INC_MIN);

                float aimCorrx = aimCorr * cosf(aimErrTh);
                float aimCorry = aimCorr * sinf(aimErrTh);

                posCmd.x = curPose.x + aimCorrx;
                posCmd.y = curPose.y + aimCorry;
                posCmd.z = curiseHeight;

                pub_spPose.publish(posCmd);

                /// if still aimming, check if the aim is good
                if (aimErrinMeter < XY_TOLLERANCE && !aimComplete) {
                    static int aimC = 0;
                    if (testMode) {
                        cout << "aimming good! times:" << aimC << endl;
                    }
                    aimC ++;
                    if (aimC > 5) {
                        cout << "aimming good more than 5 times!" << endl;
                        if (testMode) {
                            cout << "[testMode] change to leaving mode" << endl;
                            mode = MODE_LEAVING_NODE;
                        }

                        NodeInstance nodeInstance(false);
                        for (const auto & th: imageInfo.exitDirs) {
                            nodeInstance.addExit(cosf(th), sinf(th), th * RAD2DEG);
                        }
                        nodeInstance.completeAdding();
                        auto arrivedGateId = nodeInstance.getMidDirClosestExit(-curMovingDIR);
                        auto newNodeMsg = nodeInstance.encode2ROSmsg(arrivedGateId,
                                curPose.x - lastNodeX, curPose.y - lastNodeY, 0.0);
                        pub_newNode.publish(newNodeMsg);
                        lastNodeY = curPose.y;
                        lastNodeX = curPose.x;

                        aimComplete = true;

                    } else {
                        aimC = 0;
                    }
                }

                break;
            }

            case MODE_LEAVING_NODE:

                aimComplete = false;

                cout << "\nat mode: MODE_LEAVING_NODE" << endl;

                findTheLineAndGiveSP();

                if (imageInfo.exitDirs.empty()) {
                    cout << "we can't see the node info, we have left" << endl;
                    mode = MODE_ON_EDGE;
                }

                break;
        }

        if (testMode) {
            ROSSLEEP(1);
        } else {
            rate.sleep();
        }
    }
}

void cb_status(const std_msgs::UInt8 & msg) {
    automony_status = msg.data;
}

void cb_px4Pose(const px4_autonomy::Position & msg) {
    curPose = msg;
    m2px = m2pxWithUnitZ / curPose.z;
    float correctionX = tan(curPose.roll) * m2px;
    float correctionY = -tan(curPose.pitch) * m2px;
    midInImgx = imageInfo.imageSizeX / 2.0f + correctionX;
    midInImgy = -imageInfo.imageSizeY / 2.0f + correctionY;
}

void cb_image(const topology_map::ImageExract & msg) {
    imageInfo = msg;

    /// turn the coor to ENU
    imageInfo.nodePosY *= -1;
    for (int i = 0; i < imageInfo.ths.size(); i++) {
        imageInfo.ths[i] *= -1;
        fixRad2nppi(imageInfo.ths[i]);
    }
    for (auto & th: imageInfo.exitDirs) {
        th *= -1;
        fixRad2nppi(th);
    }
}

void cb_gateMove(const topology_map::LeaveNode &msg) {
    if (aimComplete) {
        mode = MODE_LEAVING_NODE;
        curMovingDIR = msg.leaveDir;
        fixRad2nppi(curMovingDIR);
    } else {
        ROS_FATAL_STREAM("\nAIMMING NOT COMPLETE\nBUT GATE MOVE ORDER SENT");
    }
}

void findTheLineAndGiveSP() {

    cerr << "[findTheLineAndGiveSP] into find line and give SP" << endl;

    /// pick the line most close to the dir
    float th = imageInfo.ths.front();
    float rh = imageInfo.rhs.front();
    float thux = cosf(th);
    float thuy = sinf(th);
    float dirInLocal = curMovingDIR - curPose.yaw; // TODO check if it's ok
    float dirux = cosf(dirInLocal);
    float diruy = sinf(dirInLocal);

    /// they are vert, so dot should be as small as possible
    float dot = fabsf(thux * dirux + thuy * diruy);
    for (int i = 1; i < imageInfo.ths.size(); i++) {
        float ath = imageInfo.ths[i];
        float athux = cosf(ath);
        float athuy = sinf(ath);
        float adot = fabsf(dirux * athux + diruy * athuy);
        if (adot < dot) {
            dot = adot;
            th = ath;
            rh = imageInfo.rhs[i];
        }
    }
    cerr << "[findTheLineAndGiveSP] following line is: th = " << th << "; rh = " << rh << endl;

    /// find the dir cloest to the desire dir
    float ldir = th;
    /// after adding pi/2, is hor not vert
    ldir += piHalf;
    float ldirux = cosf(ldir);
    float ldiruy = sinf(ldir);
    float dotld = ldirux * dirux + ldiruy * diruy;
    /// which means that we guess wrong direction
    if (dotld < 0) {
        ldir += pi;
    }
    fixRad2nppi(ldir);
    cerr << "[findTheLineAndGiveSP] I think the dir is: " << ldir << endl;

    ldir += curPose.yaw;

    float mainIncx = XY_INC_MAX * cosf(ldir);
    float mainIncy = XY_INC_MAX * sinf(ldir);
    cerr << "[findTheLineAndGiveSP] main inc: " << mainIncx << ":" << mainIncy << endl;

    /// cal the correction to follow the line

    cerr << "[findTheLineAndGiveSP] mid point: " << midInImgx << " : " << midInImgy << endl;

    if (rh < 0) {
        rh = -rh;
        th += pi;
    }

    float errorInMeterInThDIR = (rh - cosf(th) * midInImgx - sinf(th) * midInImgy) / m2px;
    /// this is PID with only P = 0.5
    float corErrInc = errorInMeterInThDIR * 0.5f;
    corErrInc = min(corErrInc, XY_INC_MIN);
    corErrInc = max(corErrInc, -XY_INC_MIN);

    th += curPose.yaw;

    float corErrx = cosf(th) * corErrInc;
    float corErry = sinf(th) * corErrInc;

    cerr << "[findTheLineAndGiveSP] cor err: " << corErrx << " : " << corErry << endl;

    posCmd.x = curPose.x + corErrx + mainIncx;
    posCmd.y = curPose.y + corErry + mainIncy;
    posCmd.z = curiseHeight;

    cerr << "[findTheLineAndGiveSP] cur pose: " << curPose.x << " : " << curPose.y << endl;
    cerr << "[findTheLineAndGiveSP] cur sp: " << posCmd.x << " : " << posCmd.y << endl;

    pub_spPose.publish(posCmd);
}
