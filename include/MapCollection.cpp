//
// Created by stumbo on 18-5-15.
//

#include "MapCollection.h"
#include "NodeInstance.h"

/**
 * the first step of arriving a node is check it in every map
 * @param instance
 * @param arriveAt
 * @param dis_x distance x since last move
 * @param dis_y distance y since last move
 */
void MapCollection::arriveNodeInstance(NodeInstance * instance, uint8_t arriveAt,
                                       double dis_x, double dis_y) {
    if (maps.empty()) {
        maps.emplace(new MapCandidate(instance));
    } else {
        for (auto & map: maps) {
            if(!map->arriveAtNode(instance, arriveAt, dis_x, dis_y)) {
                delete map;
                maps.erase(map);
            }
        }
    }
}

void MapCollection::addNewMap(MapCandidate * newMap) {
    maps.insert(newMap);
}
