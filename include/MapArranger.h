//
// Created by stumbo on 18-5-21.
//

#ifndef TOPOLOGY_MAP_MAPARRANGER_H
#define TOPOLOGY_MAP_MAPARRANGER_H

#include "TopoType.h"
#include "MapCollection.h"
#include "NodeCollection.h"

class NodeInstance;

/**
 * A boss class of the map part
 */
class MapArranger {
public:
    void arriveInstance(NodeInstance * instance, gateId arriveAt,
                        double dis_x, double dis_y);
    void moveThroughGate(gateId gate);
    size_t getMapNumbers();
private:
    NodeCollection nodeCollection;
    MapCollection mapCollection;
};


#endif //TOPOLOGY_MAP_MAPARRANGER_H
