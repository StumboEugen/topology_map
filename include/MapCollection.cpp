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
        auto firstMap = new MapCandidate(instance);
        maps.emplace_back(firstMap);
        firstMap->setPosInList(maps.begin());
    } else {
        auto iter = maps.begin();
        while (iter != maps.end()) {
            auto & map = *iter;
            if(!map->arriveAtNode(instance, arriveAt, dis_x, dis_y)) {  //TODO sort pos
                delete map;
                iter = maps.erase(iter);
            } else {
                iter ++;
            }
        }
    }
}

list<MapCandidate *>::iterator
MapCollection::addNewMap(list<MapCandidate *>::iterator pos2Insert, MapCandidate *newMap) {
    return maps.insert(pos2Insert, newMap);
}

void MapCollection::everyMapThroughGate(gateId exit) {
    for (const auto & map: maps) {
        map->setLeaveFrom(exit);
    }
}
