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

JSobj MapCollection::toJSWithSortedMaps(size_t mapCount) {
    JSobj obj;
    if (mapCount == 0 || mapCount > maps.size()) {
        mapCount = maps.size();
    }
    sortByConfidence(mapCount);
    for (int i = 0; i < mapCount; i++) {
        obj.append(orderedMaps[i]->toJS());
    }
    return std::move(obj);
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

void MapCollection::sortByConfidence(size_t topCount) {
    size_t experience = parent->getNodeCollection().experienceSize();
    auto comp = [experience](MapCandidate * a, MapCandidate * b){
        return a->getConfidence(experience) > b->getConfidence(experience) ;
    };
    orderedMaps.clear();
    orderedMaps.reserve(maps.size());
    orderedMaps.assign(maps.size(), nullptr);
    copy(maps.begin(), maps.end(), orderedMaps.begin());
    if (topCount == 0 || topCount > maps.size()) {
        topCount = maps.size();
    }
    partial_sort(orderedMaps.begin(), orderedMaps.begin() + topCount, orderedMaps.end(), comp);
}


void MapCollection::purgeBadMaps(int survival) {
    if (survival > maps.size()) {
        survival = static_cast<int>(maps.size());
    }

    sortByConfidence(static_cast<size_t>(survival));

    for (int i = survival; i < orderedMaps.size(); i++) {
        auto map2delete = orderedMaps[i];
        maps.erase(map2delete);
        map2delete->detachAllInstances();
        delete map2delete;
    }

    orderedMaps.erase(orderedMaps.begin() + survival, orderedMaps.end());
}

MapCollection::MapCollection(MapArranger *parent)
        :parent(parent)
{

}

void MapCollection::calSumOfConfidence() {
    sumOfConfidence = 0;
    for (const auto & map: maps) {
        sumOfConfidence += map->getConfidence(parent->getNodeCollection().experienceSize());
    }
}
