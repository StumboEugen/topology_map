//
// Created by stumbo on 18-5-21.
//

#include <iostream>
#include <vector>

#include "NodeCollection.h"
#include "NodeInstance.h"
#include "MapCandidate.h"

using namespace std;

/**
 * compare the instace with every node and find the similiar ones
 * @param instance
 * @param arriveAt
 * @param dis_x
 * @param dis_y
 * @return the new map caused by the connection to similiar node
 */
vector<MapCandidate*> NodeCollection::addInstanceAndCompare(NodeInstance *instance, uint8_t arriveAt,
                                           double dis_x, double dis_y) {

    vector<MapCandidate*> newMaps;
    auto & nodeSet = nodeSets[instance->sizeOfExits()];
    auto iter = nodeSet.begin();
    while (iter != nodeSet.end()) {
        if ((*iter)->haveNoUseages()) {
            iter = nodeSet.erase(iter);
        } else {
            auto & nodeInstance = *iter;
            if (nodeInstance->alike(*instance)) {
                for (auto & useage : nodeInstance->getNodeUseages()) {
                    if (!useage.first->isJustMovedOnKnownEdge()) {
                        auto newMap = useage.first->arriveAtSimiliar(useage.second, arriveAt);
                        if (newMap != nullptr) {
                            newMaps.push_back(newMap);
                        }
                    }
                }
            }
            iter++;
        }
    }

    return std::move(newMaps);
}
