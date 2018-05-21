//
// Created by stumbo on 18-5-15.
//

#include "MapCollection.h"
#include "NodeInstance.h"

void MapCollection::arriveNodeInstance(const NodeInstance * instance, uint8_t arriveAt,
                                       double dis_x, double dis_y) {
    for (auto & map: maps) {
        map->arriveAtNode(instance, arriveAt, dis_x, dis_y);
    }
}
