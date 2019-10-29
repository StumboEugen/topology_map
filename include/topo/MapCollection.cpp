//
// Created by stumbo on 18-5-15.
//

#include "Topo.h"

/**
 * @brief deduce every MapCandidate in \b this to arrive at next NodeInstance.
 * the first step of arriving a node is deducing it in every map.
 * some conflicts may happen and the map can be ruled out.
 * if the edge moving on has never moved on, we consider the next node is a brand new node here
 * @param instance the new NodeInstance just arrived
 * @param arriveAt the gate just arrived at
 * @param dis_x distance x since last move
 * @param dis_y distance y since last move
 * @param yaw orientation since last move (unused)
 */
void MapCollection::arriveNodeInstance(NodeInstance *instance, uint8_t arriveAt,
                                       double dis_x, double dis_y, double yaw) {
    if (maps.empty()) {
        auto firstMap = new MapCandidate(instance);
        maps.insert(firstMap);
    } else {

        MapCandidate* currentPathRelatedMap = currentPath.getRelatedMap();

        auto iter = maps.begin();
        while (iter != maps.end()) {
            auto & map = *iter;
            if(!map->arriveAtNode(instance, arriveAt, dis_x, dis_y, yaw)) {

                if (currentPathRelatedMap == map)
                {
                    currentPath.setInvalid();
                }

                map->removeUseages();
                delete map;
                iter = maps.erase(iter);
            } else {
                iter ++;
            }
        }
    }
}

/**
 * @brief directly add a new MapCandidate
 * this is used to build MapCollection in other situation
 * @param newMap the new MapCandidate
 */
void MapCollection::addNewMap(MapCandidate *newMap) {
    maps.insert(newMap);
}

/**
 * @brief set every MapCandidate 's robot to move throuth a gate.
 * @param exit the gate to move through
 */
void MapCollection::everyMapThroughGate(gateId exit) {
    for (const auto & map: maps) {
        map->setLeaveFrom(exit);
    }
}

/**
 * @brief convert MapCollection to JSON structure with a limited amount of maps
 * the maps with highest confidence would be converted.
 * @param mapCount the count of map you would like to save (0 == whole)
 * @see MapCollection::toJS()
 */
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

/**
 * @brief convert MapCollection to JSON structure
 * @return
 * [] array of MapCandidate
 * @see MapCandidate::toJS()
 */
JSobj MapCollection::toJS() const {
    JSobj obj;
    for (const auto & mapIns: maps) {
        obj.append(mapIns->toJS());
    }
    return std::move(obj);
}

/**
 * @brief clear all MapCandidate s.
 * @see MapCandidate::~MapCandidate()
 * @warning the usages in NodeInstance would not be cleared, because the ptr of NodeInstance
 * may has been deleted.
 */
void MapCollection::clear() {
    for (const auto & map: maps) {
        delete map;
    }
    maps.clear();
    orderedMaps.clear();
}

/**
 * @brief add TopoNode to MapCollection directly.
 * this is used for UI building mode
 */
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

/**
 * @brief add TopoEdge to MapCollection directly.
 * this is used for UI building mode
 */
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

/**
 * sort the map according to confidence. the sorted map will be stored in
 * \link orderedMaps \endlink.
 * @param topCount the amount of maps to sort
 * @attention the \link orderedMaps \endlink orderedMaps contains all maps and the topCount
 * of map are sorted
 */
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

/**
 * @brief purge assigned number of maps according to the confidence.
 * @param survivorCount the amount of survivors
 */
void MapCollection::purgeBadMaps(int survivorCount) {
    if (survivorCount > maps.size()) {
        survivorCount = static_cast<int>(maps.size());
    }

    sortByConfidence(static_cast<size_t>(survivorCount));

    for (int i = survivorCount; i < orderedMaps.size(); i++) {
        auto map2delete = orderedMaps[i];
        map2delete->removeUseages();
        maps.erase(map2delete);
        delete map2delete;
    }

    orderedMaps.erase(orderedMaps.begin() + survivorCount, orderedMaps.end());
}

/**
 * @brief constructor of MapCollection.
 * @param parent the MapArranger that contains this
 */
MapCollection::MapCollection(MapArranger *parent)
        :parent(parent)
{

}

/**
 * @brief calculate the sum of confidence
 * this is for calculate the normalized posibility
 * @return the sum
 * @note the sum will be stored in MapCollection::sumOfConfidence
 */
double MapCollection::calSumOfConfidence() {
    sumOfConfidence = 0;
    for (const auto & map: maps) {
        sumOfConfidence += map->getConfidence(parent->getNodeCollection().experienceSize());
    }
    return sumOfConfidence;
}
