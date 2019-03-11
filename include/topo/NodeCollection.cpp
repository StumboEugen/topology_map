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
 * the new maps caused by moving to similiar nodes will be returned
 * @param instance
 * @param arriveAt
 * @param dis_x
 * @param dis_y
 * @return the new maps caused by moving to similiar nodes
 */
vector<MapCandidate *> NodeCollection::addInstanceAndCompare
        (NodeInstance *instance, uint8_t arriveAt, double dis_x, double dis_y) {

    vector<MapCandidate *> newMaps;
    vector<pair<MapCandidate *, TopoNode*>> loopDetected;
    /**find data group we need to check*/
    auto & nodeSet = nodeSets[instance->sizeOfExits()];
    auto iter = nodeSet.begin();
    while (iter != nodeSet.end()) {
        if ((*iter)->haveNoUseages()) {
            iter = nodeSet.erase(iter);
        } else {
            auto & nodeInstance = *iter;
            /**they are alike! check the useage in map, if it is propriate*/
            if (nodeInstance->alike(*instance)) {
                for (auto & useage : nodeInstance->getNodeUseages()) {
                    MapCandidate * relatedMap = useage.first;
                    if (!relatedMap->isJustMovedOnKnownEdge()) {
                        loopDetected.emplace_back(relatedMap, useage.second);
                    }
                }
            }
            iter++;
        }
    }

    for (auto & loopCondition: loopDetected) {
        auto newMap = loopCondition.first->arriveAtSimiliar(loopCondition.second, arriveAt);
        if (newMap != nullptr) {
            newMaps.push_back(newMap);
        }
    }

    nodeSet.insert(instance);

    return std::move(newMaps);
}

JSobj NodeCollection::toJS() const {
    JSobj obj;
    for (const auto & nodeSet: nodeSets) {
        for (const auto & node: nodeSet.second) {
            obj.append(node->toJS());
        }
    }
    return std::move(obj);
}

void NodeCollection::clear() {
    for (const auto & nodeSet: nodeSets) {
        for (const auto & node: nodeSet.second) {
            delete node;
        }
    }
    nodeSets.clear();
}

void NodeCollection::addInstanceDirectly(NodeInstance *newNode) {
    nodeSets[newNode->sizeOfExits()].insert(newNode);
}
