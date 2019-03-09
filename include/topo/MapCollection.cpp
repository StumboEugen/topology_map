//
// Created by stumbo on 18-5-15.
//

#include "Topo.h"

/**
 * the first step of arriving a node is check it in every map
 * @param instance
 * @param arriveAt
 * @param dis_x distance x since last move
 * @param dis_y distance y since last move
 */
void MapCollection::arriveNodeInstance(NodeInstance *instance, uint8_t arriveAt,
                                       double dis_x, double dis_y, double yaw) {
    if (maps.empty()) {
        auto firstMap = new MapCandidate(instance);
        maps.insert(firstMap);
//        firstMap->setPosInList(maps.begin());
    } else {
        auto iter = maps.begin();
        while (iter != maps.end()) {
            auto & map = *iter;
            if(!map->arriveAtNode(instance, arriveAt, dis_x, dis_y, yaw)) {  //TODO sort pos
                map->removeUseages();
                delete map;
                iter = maps.erase(iter);
            } else {
                iter ++;
            }
        }
    }
}

void MapCollection::addNewMap(MapCandidate *newMap) {
    maps.insert(newMap);
}

void MapCollection::everyMapThroughGate(gateId exit) {
    for (const auto & map: maps) {
        map->setLeaveFrom(exit);
    }
}

JSobj MapCollection::toJS() const {
    JSobj obj;
    for (const auto & mapIns: maps) {
        obj.append(mapIns->toJS());
    }
    return std::move(obj);
}

void MapCollection::clear() {
    for (const auto & map: maps) {
        delete map;
    }
    maps.clear();
}

void MapCollection::addNodeDirectly(TopoNode * node)  {
    if (maps.size() > 1) {
        std::cerr << "[MapCollection::addNodeDirectly]"
                     "You try to add node directly, but there are more than 1 map candidate"
                  << endl;
    }

    if (maps.empty()) {
        maps.insert(new MapCandidate(node->getInsCorrespond()));
    }

    (*maps.begin())->addNodeDirectly(node);
}

void MapCollection::addEdgeDirectly(TopoEdge * edge) {
    if (maps.size() > 1) {
        std::cerr << "[MapCollection::addNodeDirectly]"
                     "You try to add node directly, but there are more than 1 map candidate"
                  << endl;
    }

    if (maps.empty()) {
        maps.insert(new MapCandidate(edge->getNodeA()->getInsCorrespond()));
    }

    (*maps.begin())->addEdgeDirectly(edge);
}
