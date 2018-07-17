//
// Created by stumbo on 18-5-21.
//

#ifndef TOPOLOGY_MAP_MAPARRANGER_H
#define TOPOLOGY_MAP_MAPARRANGER_H

#include "MapCollection.h"
#include "NodeCollection.h"

/**
 * A boss class of the map part
 */
class MapArranger {
public:
    void arriveInstance(NodeInstance * instance, uint8_t arriveAt,
                        double dis_x, double dis_y);
private:
    NodeCollection nodeCollect;
    MapCollection mapCollect;
};


#endif //TOPOLOGY_MAP_MAPARRANGER_H
