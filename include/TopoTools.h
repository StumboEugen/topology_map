//
// Created by stumbo on 18-7-18.
//

#ifndef TOPOLOGY_MAP_TOPOTOOL_H
#define TOPOLOGY_MAP_TOPOTOOL_H

#include <cstdint>
#include <list>
#include <string>

typedef uint8_t	gateId;

class MapCandidate;
typedef std::list<MapCandidate *>::iterator mapPosInList;

static const double piHalf = 3.1415926 / 2.0;
static const double pi = 3.1415926;
static const double piTwo = 3.1415926 * 2.0;

#define TOPO_STD_FILE_SAVE_FLODER_NAME "topoMaps/"

#define TOPO_STD_TOPIC_NAME_NODEINFO "Topo/NewTravel"
#define TOPO_STD_TOPIC_NAME_GATEMOVE "Topo/MoveThroughGate"
#define TOPO_STD_SERVICE_NAME_SAVEMAP "TopoSrv/SaveMap"

namespace topo {
    const std::string getCurrentTimeString();
}

#endif //TOPOLOGY_MAP_TOPOTOOL_H
