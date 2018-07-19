//
// Created by stumbo on 18-5-21.
//

#include <iostream>
#include <vector>
#include <list>
#include <set>


#include "NodeCollection.h"
#include "NodeInstance.h"
#include "MapCandidate.h"

using namespace std;

/**
 * compare the instace with every node and find the similiar ones
 * the new maps caused by moving to similiar node will be stored in the map fathers
 * @param instance
 * @param arriveAt
 * @param dis_x
 * @param dis_y
 * @return
 */
vector<pair<std::list<MapCandidate *>::iterator, MapCandidate *>>
NodeCollection::addInstanceAndCompare
        (NodeInstance *instance, uint8_t arriveAt, double dis_x, double dis_y) {
    vector<pair<std::list<MapCandidate *>::iterator, MapCandidate *>> newMaps;
    auto & nodeSet = nodeSets[instance->sizeOfExits()];
    auto iter = nodeSet.begin();
    while (iter != nodeSet.end()) {
        if ((*iter)->haveNoUseages()) {
            iter = nodeSet.erase(iter);
        } else {
            auto & nodeInstance = *iter;
            if (nodeInstance->alike(*instance)) {
                for (auto & useage : nodeInstance->getNodeUseages()) {
                    MapCandidate * relatedMap = useage.first;
                    if (!relatedMap->isJustMovedOnKnownEdge()) {
                        auto newMap = relatedMap->arriveAtSimiliar(useage.second, arriveAt);
                        if (newMap != nullptr) {
                            newMaps.emplace_back(relatedMap->getListPosition(), newMap);
                        }
                    }
                }
            }
            iter++;
        }
    }

    nodeSet.insert(instance);

    return std::move(newMaps);
}
