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
    for (auto & node : nodeSet) {
        if (*node == *instance) {
            for (auto & useage : node->getNodeUseages()) {
                auto newMap = useage.first->arriveAtSimiliar(useage.second, arriveAt);
                if (newMap != nullptr) {
                    newMaps.push_back(newMap);
                }
            }
        }
    }
    return std::move(newMaps);
}
