//
// Created by stumbo on 18-10-17.
//

#ifndef TOPOLOGY_MAP_UITOOLS_H
#define TOPOLOGY_MAP_UITOOLS_H

const uint32_t INS_TYPE_SERIAL = 1074;

const int METER_TO_PIXLE = 120;
const int QNODE_CIRCLE_SIZE = METER_TO_PIXLE / 4;
const double SCALE_TIME = 1.2;
const double SCALE_MAX = 10.0;

enum UIMode {
    READ_MODE,
    BUILD_MODE,
    SIMULATION_MODE,
    REALTIME_MODE,
};

#endif //TOPOLOGY_MAP_UITOOLS_H
