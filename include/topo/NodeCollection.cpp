//
// Created by stumbo on 18-5-21.
//

#include <iostream>
#include <vector>
#include <list>
#include <set>
#include <map>
#include <tuple>

#include "NodeCollection.h"
#include "NodeInstance.h"
#include "MapCandidate.h"

using namespace std;

/**
 * compare the instace with every node and find the similiar ones
 * the new maps caused by moving to similiar nodes will be returned
 * @param newIns
 * @param arriveAt
 * @param dis_x
 * @param dis_y
 * @return the new maps caused by moving to similiar nodes
 */
vector<MapCandidate *> NodeCollection::addInstanceAndCompare
        (NodeInstance *newIns, uint8_t arriveAt, double dis_x, double dis_y) {


    if (experiences.empty()) {
        newIns->setGlobalPos(0.0, 0.0, 0.0);
    } else {
        double dis = sqrt(dis_x * dis_x + dis_y * dis_y);
        const auto lastIns = experiences.back();
        newIns->setGlobalPos(dis_x + lastIns->getGlobalX(),
                             dis_y + lastIns->getGlobalY(),
                             dis + lastIns->getTravelDis());
    }
    experiences.push_back(newIns);

    vector<MapCandidate *> newMaps;

    // TODO use faster structure
    /// alike xxx instance, with useage of xxx map at xxx node with xxx diff
    map< NodeInstance*, vector<tuple<MapCandidate *, TopoNode*, int>>> loopDetected;
    /**find data group we need to check*/
    auto & nodeSet = nodeSets[newIns->sizeOfExits()];
    auto iter = nodeSet.begin();
    while (iter != nodeSet.end()) {
        /// 20190408 no useages node ins is still required in the experiences
//        if ((*iter)->haveNoUseages()) {
//            iter = nodeSet.erase(iter);
//        } else {
            auto & nodeInstance = *iter;
            /**they are alike! check the useage in map, if it is propriate*/
            int diff = newIns->alike(*nodeInstance);
            if (diff >= 0) {
                for (const auto & useage : nodeInstance->getNodeUseages()) {
                    MapCandidate * relatedMap = useage.first;
                    if (!relatedMap->isJustMovedOnKnownEdge()) {
                        loopDetected[nodeInstance].emplace_back(
                                relatedMap, useage.second, diff);
                    }
                }
            }
            iter++;
//        }
    }

    /// do the arrive at similiar task at here, if earlier, the ins useages would be polluted
    for (auto & alikeInsPair: loopDetected) {
        auto & alikeIns = alikeInsPair.first;

        /// calculate the fixCoe according to the odom along the walking path
        double travelDis = newIns->getTravelDis() - alikeIns->getTravelDis();
        double error = topo::calDis(
                alikeIns->getGlobalX() - newIns->getGlobalX(),
                alikeIns->getGlobalY() - newIns->getGlobalY());
        double stdError = error / travelDis;
        double fixCoe = exp(-0.5 * stdError * stdError / convDistPerMeter);
        ///////////////////////////

        for (auto & loopTuple : alikeInsPair.second) {
            const auto diff = get<2>(loopTuple);
            auto modifiedExitNum = static_cast<uint8_t>(arriveAt + diff);
            modifiedExitNum %= newIns->sizeOfExits();
            auto newMap = get<0>(loopTuple)
                    ->arriveAtSimiliar(get<1>(loopTuple), modifiedExitNum, diff);
            if (newMap != nullptr) {
                newMap->xConfidence(fixCoe); //TODO
                newMaps.push_back(newMap);
            }
        }
    }

    nodeSet.insert(newIns);

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
    experiences.clear();
    nodeSets.clear();
}

void NodeCollection::addInstanceDirectly(NodeInstance *newNode) {
    nodeSets[newNode->sizeOfExits()].insert(newNode);
    experiences.push_back(newNode);
}

NodeCollection::NodeCollection(MapArranger *parent):
        parent(parent)
{

}
