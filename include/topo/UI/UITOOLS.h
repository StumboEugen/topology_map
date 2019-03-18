//
// Created by stumbo on 18-10-17.
//

#ifndef TOPOLOGY_MAP_UITOOLS_H
#define TOPOLOGY_MAP_UITOOLS_H

const uint32_t INS_TYPE_SERIAL = 1074;

const int METER_TO_PIXLE = 30;

const double PI = 3.1415926;
const double DEG2RAD = PI / 180;
const double RAD2DEG = 180 / PI;

enum UIMode {
    READ_MODE,
    BUILD_MODE,
    SIMULATION_MODE,
    REALTIME_MODE,
};

#endif //TOPOLOGY_MAP_UITOOLS_H
