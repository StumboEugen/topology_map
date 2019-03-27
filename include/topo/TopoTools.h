//
// Created by stumbo on 18-7-18.
//

#ifndef TOPOLOGY_MAP_TOPOTOOL_H
#define TOPOLOGY_MAP_TOPOTOOL_H

#include <cstdint>
#include <list>
#include <string>
#include <vector>
#include <cmath>

#include "json/json.h"

typedef uint8_t	gateId;

class MapCandidate;
class NodeInstance;
class TopoNode;
class TopoEdge;
class ExitInstance;

typedef Json::Value JSobj;

static const double convEdgePerMeter = 0.02;
static const double stdDevEdgePerMeter = 0.1 * M_SQRT2;
static const double stdDevEdgePerMeterOneAx = 0.1;
static const double convEdgePerMeterOneAx = 0.01;
/// in RAD
static const double convNodePerGate = 0.04;

static const double convDistPerMeter = 0.01;

static const double piHalf = 3.1415926 / 2.0;
static const double pi = 3.1415926;
static const double piTwo = 3.1415926 * 2.0;

static const double DEG2RAD = pi / 180;
static const double RAD2DEG = 180 / pi;

#define TOPO_STD_FILE_SAVE_FLODER_NAME "topoMaps/"

#define TOPO_STD_TOPIC_NAME_NODEINFO "topo/ArriveAtNewNode"
#define TOPO_STD_TOPIC_NAME_GATEMOVE "topo/LeaveFromNode"
#define TOPO_STD_TOPIC_NAME_CVINFO "topo/cvInfo"
#define TOPO_STD_SERVICE_NAME_SAVEMAP "topoSrv/SaveMap"
#define TOPO_STD_SERVICE_NAME_GETMAPS "topoSrv/GetMaps"

namespace topo {
    const std::string getCurrentTimeString();
    bool checkJSMember(const std::vector<std::string> &strs, const JSobj &js);
    double calDis(double x, double y);
    double fixRad2nppi(double target);
}

#endif //TOPOLOGY_MAP_TOPOTOOL_H
