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
bool testMode = false;

/// false: direct pass the aimming mode
bool waitForNewGateCmd = true;

int main(int argc, char **argv) {

    cout << "test mode: " << testMode << "\n" << "waitForNewGateCmd: " << waitForNewGateCmd << endl;

    ros::init(argc, argv, "localController");
    ros::NodeHandle n;

    ros::Subscriber sub_px4pose = n.subscribe("/px4/pose", 1, cb_px4Pose);
    ros::Subscriber sub_status = n.subscribe("/px4/status", 1, cb_status);
    ros::Subscriber sub_image = n.subscribe(TOPO_STD_TOPIC_NAME_CVINFO, 1, cb_image);
    ros::Subscriber sub_gateMove = n.subscribe(TOPO_STD_TOPIC_NAME_GATEMOVE, 1, cb_gateMove);

    pub_spPose = n.advertise<px4_autonomy::Position>("/px4/cmd_pose", 1);
    pub_takeOff = n.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1);
    pub_newNode = n.advertise<topology_map::NewNodeMsg>(TOPO_STD_TOPIC_NAME_NODEINFO, 1);

    ROSSLEEP(0.5);
    ros::spinOnce();

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

        ROSSLEEP(2);

        move2Z(curiseHeight);
    } else {
        m2px = m2pxWithUnitZ / 0.5f;
        midInImgx = 320;
        midInImgy = -240;
    }

    posCmd.yaw = curPose.yaw;
        
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
                    cout << "node X" << imageInfo.nodePosX << "\t" << imageInfo.nodePosY << endl;
                    cout << "the node dir is " << thOfNode << endl;
                    const auto diffABS = fabsf(thOfNode - (curMovingDIR + curPose.yaw - piHalf)); 
                    cout << "diff: " << diffABS << endl;
                    if (diffABS < piHalf || fabsf(diffABS - piTwo) < piHalf) {
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

                if (imageInfo.nodePosX < 0) {

                    cerr << "NO CROSS NODE DETECTED AT AIMMING MODE!" << endl;

                    static int errorCount = 0;

                    if (errorCount++ == 0) {
                        curSP.x = curPose.x;
                        curSP.y = curPose.y;
                    }

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

                float aimErrTh = atan2f(aimErry, aimErrx) + curPose.yaw - piHalf;

                /// TODO  check if the coor is good in meter
                float aimCorr = slopeCal(aimErrinMeter, 0.05, XY_SAFEDIS, 0.0, 0.1);

                static float lastAimCorr;
                float aimCorrD = lastAimCorr - aimCorr;
                aimCorr -= aimCorrD * 0.3;
                lastAimCorr = aimCorr;

                float aimCorrx = aimCorr * cosf(aimErrTh);
                float aimCorry = aimCorr * sinf(aimErrTh);

                curSP.x = curPose.x + aimCorrx;
                curSP.y = curPose.y + aimCorry;
                curSP.z = curiseHeight;

                curSP.loadXYZ2POS(posCmd);
                pub_spPose.publish(posCmd);

                cout << "[AIMMING MODE] err in px" << aimErrx << "\t" << aimErry << endl;
                cout << "[AIMMING MODE] err in meter" << aimErrinMeter << endl;
                cout << "[AIMMING MODE] XY_TOLLERANCE:" << XY_TOLLERANCE << endl;
                cout << "[AIMMING MODE] cor sp +++:" << aimCorrx << "\t" << aimCorry << endl;

                static int aimC = 0;
                /// if still aimming, check if the aim is good
                if (aimErrinMeter < XY_TOLLERANCE && !aimComplete
                    && !imageInfo.exitDirs.empty()) {
                    
                    if (testMode) {
                        cout << "aimming good! times:" << aimC << endl;
                    }
                    aimC ++;
                    if (aimC > 5) {
                        aimC = 0;
                        cout << "aimming good more than 5 times!" << endl;
                        if (!waitForNewGateCmd) {
                            cout << "[testMode] change to leaving mode" << endl;
                            mode = MODE_LEAVING_NODE;
                        }

                        NodeInstance nodeInstance(false);
                        for (const auto & th: imageInfo.exitDirs) {
                            nodeInstance.addExit(cosf(th), sinf(th), th * RAD2DEG);
                        }
                        nodeInstance.completeAdding();
                        auto arrivedGateId = nodeInstance.getMidDirClosestExit(fixRad2nppi(curMovingDIR + pi));
                        auto newNodeMsg = nodeInstance.encode2ROSmsg(arrivedGateId,
                                curPose.x - lastNodeX, curPose.y - lastNodeY, 0.0);
                        pub_newNode.publish(newNodeMsg);
                        lastNodeY = curPose.y;
                        lastNodeX = curPose.x;

                        aimComplete = true;
                    }
                } else {
                    aimC = 0;
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
    float correctionX = tan(curPose.roll) * curPose.z * m2px * 0.7f;
    float correctionY = -tan(curPose.pitch) * curPose.z * m2px * 0.7f;
//    cout << "[cb_px4Pose] m2px: " << m2px << endl;
//    cout << "[cb_px4Pose] rp: " << curPose.roll << "\t" << curPose.pitch << endl;
//    cout << "[cb_px4Pose] correction: " << correctionX << "\t" << correctionY << endl;
    midInImgx = imageInfo.imageSizeX / 2.0f + correctionX;
    midInImgy = -imageInfo.imageSizeY / 2.0f + correctionY;
}

void cb_image(const topology_map::ImageExract & msg) {
    imageInfo = msg;

    /// turn the coor to ENU
    imageInfo.nodePosY *= -1;
    for (int i = 0; i < imageInfo.ths.size(); i++) {
        imageInfo.ths[i] = fixRad2nppi(imageInfo.ths[i] * -1.0);
    }
    for (auto & th: imageInfo.exitDirs) {
        th = fixRad2nppi(th * -1);
    }
}

void cb_gateMove(const topology_map::LeaveNode &msg) {
    cout << "[cb_gateMove] I got the leave Dir: " << msg.leaveDir << endl;
    cout << "[cb_gateMove] I got the leave Gate: " << msg.leaveGate << endl;
    if (aimComplete) {
        mode = MODE_LEAVING_NODE;
        curMovingDIR = fixRad2nppi(msg.leaveDir);
    } else {
        ROS_FATAL_STREAM("\nAIMMING NOT COMPLETE\nBUT GATE MOVE ORDER SENT");
    }
}

void findTheLineAndGiveSP() {

    /// pick the line most close to the dir
    float th = imageInfo.ths.front();
    float rh = imageInfo.rhs.front();
    float thux = cosf(th);
    float thuy = sinf(th);
    float dirInLocal = curMovingDIR - (curPose.yaw - piHalf); // TODO check if it's ok
    cerr << "[findTheLineAndGiveSP] Yaw in global is: " << curPose.yaw << endl;
    cerr << "[findTheLineAndGiveSP] Dir target(in global): " << curMovingDIR << endl;
    cerr << "[findTheLineAndGiveSP] Dir target(in img): " << dirInLocal << endl;
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
    cerr << "[findTheLineAndGiveSP] following line's th: " << th << endl;

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
    ldir = fixRad2nppi(ldir);
    cerr << "[findTheLineAndGiveSP] I think the dir is(in img): " << ldir << endl;

    ldir += curPose.yaw - piHalf;

    cerr << "[findTheLineAndGiveSP] I think the dir is(in global): " << ldir << endl;

    float mainIncx = XY_INC_MIN * cosf(ldir);
    float mainIncy = XY_INC_MIN * sinf(ldir);
    if (mode == MODE_ARRIVING_NODE) {
        mainIncx *= 0.6;
        mainIncy *= 0.6;
    }
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
    // corErrInc *= 0.8;

    th += curPose.yaw - piHalf;

    float corErrx = cosf(th) * corErrInc;
    float corErry = sinf(th) * corErrInc;

    cerr << "[findTheLineAndGiveSP] cor err: " << corErrx << " : " << corErry << endl;

    curSP.x = curPose.x + corErrx + mainIncx;
    curSP.y = curPose.y + corErry + mainIncy;
    curSP.z = curiseHeight;

    cerr << "[findTheLineAndGiveSP] cur sp+++: " << corErrx + mainIncx << " : " << corErry + mainIncy << endl;

    curSP.loadXYZ2POS(posCmd);
    pub_spPose.publish(posCmd);
}
