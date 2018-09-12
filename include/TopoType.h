//
// Created by stumbo on 18-7-18.
//

#ifndef TOPOLOGY_MAP_TOPOTYPE_H
#define TOPOLOGY_MAP_TOPOTYPE_H

#include <cstdint>
#include <list>

typedef uint8_t	gateId;

class MapCandidate;
typedef std::list<MapCandidate *>::iterator mapPosInList;

static const double piHalf = 3.1415926 / 2.0;
static const double pi = 3.1415926;
static const double piTwo = 3.1415926 * 2.0;

#define TOPO_STD_TOPIC_NAME_NODEINFO "Topo/NewTravel"
#define TOPO_STD_TOPIC_NAME_GATEMOVE "Topo/MoveThroughGate"

#endif //TOPOLOGY_MAP_TOPOTYPE_H
