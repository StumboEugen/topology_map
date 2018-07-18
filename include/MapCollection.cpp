//
// Created by stumbo on 18-5-15.
//

#include "TopoMap.h"

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
        auto iter = maps.begin();
        while (iter != maps.end()) {
            auto & map = *iter;
            if(!map->arriveAtNode(instance, arriveAt, dis_x, dis_y)) {
                delete map;
                iter = maps.erase(iter);
            } else {
                iter ++;
            }
        }
    }
}

void MapCollection::addNewMap(MapCandidate * newMap) {
    maps.insert(newMap);
}

void MapCollection::everyMapThroughGate(gateId exit) {
    for (const auto & map: maps) {
        map->setLeaveFrom(exit);
    }
}
