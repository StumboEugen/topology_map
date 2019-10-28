//
// Created by stumbo on 18-5-21.
//

#include <vector>
#include <list>
#include <map>
#include <tuple>

#include "NodeCollection.h"
#include "NodeInstance.h"
#include "MapCandidate.h"

using namespace std;

/**
 * compare the instances with every node and return the maps happen loop-closure.
 * the new maps caused by moving to similiar nodes will be returned.
 * @param newIns the newly arrived NodeInstance
 * @param arriveAt the gate that arrived at
 * @param dis_x the odom movement in X-axis
 * @param dis_y the odom movement in Y-axis
 * @return the new maps caused by moving to similiar nodes
 *
 * @attention the new NodeInstance 's owner is passed to this NodeCollection
 */
vector<MapCandidate *> NodeCollection::addInstanceAndCompare
        (NodeInstance *newIns, uint8_t arriveAt, double dis_x, double dis_y) {


    ///////////////////// add nodeInstance to vector collection

    if (experiences.empty()) {
        newIns->setGlobalPos(0.0, 0.0, 0.0);
    } else {
        double dis = sqrt(dis_x * dis_x + dis_y * dis_y);
        const auto & lastIns = experiences.back();
        newIns->setGlobalPos(dis_x + lastIns->getGlobalX(),
                             dis_y + lastIns->getGlobalY(),
                             dis + lastIns->getTravelDis());
    }

    newIns->setSerialNumber(experiences.size());
    experiences.push_back(newIns);

    ///////////////////// find similiar NodeInstance s

    /// alike xxx instance, with useage of xxx map at xxx node with xxx diff
    /// @todo use faster structure
    map< NodeInstance*, vector<tuple<MapCandidate *, TopoNode*, int>>> loopDetected;
    /**find data group we need to check*/
    auto & nodeSet = nodeSets[newIns->sizeOfExits()];
    auto iter = nodeSet.begin();
    while (iter != nodeSet.end()) {
        auto & oldInstance = *iter;
        if (oldInstance->haveNoUseages()) {
            iter = nodeSet.erase(iter);
        } else {
            /**they are alike! check the useage in map, if it is propriate*/
            int diff = newIns->alike(*oldInstance);
            if (diff >= 0) {
                for (const auto & useage : oldInstance->getNodeUseages()) {
                    MapCandidate * relatedMap = useage.first;
                    if (!relatedMap->isJustMovedOnKnownEdge()) {
                        loopDetected[oldInstance].emplace_back(
                                relatedMap, useage.second, diff);
                    }
                }
            }
            iter++;
        }
    }

    ////////////////////// check if loop-closure availabile

    vector<MapCandidate *> newMaps;
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

/**
 * @brief convert the NodeCollection to JSON structure
 * @return
 * [] array of NodeInstance
 * @see NodeInstance::toJS()
 * @note the order of time is implyed in the serial number
 */
JSobj NodeCollection::toJS() const {
    JSobj obj;

    for (size_t i = 0; i < experiences.size(); ++i)
    {
        auto nodeJS = experiences[i]->toJS();
        nodeJS["No"] = i;
        obj.append(std::move(nodeJS));
    }

//    for (const auto & nodeSet: nodeSets) {
//        for (const auto & node: nodeSet.second) {
//            obj.append(node->toJS());
//        }
//    }
    return std::move(obj);
}

/**
 * @brief clear all info and destroy NodeInstances.
 * @attention NodeInstance would be delete
 */
void NodeCollection::clear() {
    for (const auto & nodeSet: nodeSets) {
        for (const auto & node: nodeSet.second) {
            delete node;
        }
    }
    experiences.clear();
    nodeSets.clear();
}

/**
 * @brief manually add a NodeInstance.
 * this is usually used for building fake map
 * @param newNode the new Node
 */
void NodeCollection::addInstanceDirectly(NodeInstance *newNode) {
    nodeSets[newNode->sizeOfExits()].insert(newNode);
    newNode->setSerialNumber(experiences.size());
    experiences.push_back(newNode);
}

/**
 * @brief the main constructor.
 * for conveient, we need NodeCollection knows the parent MapArranger
 * @param parent the MapArranger that contains this
 */
NodeCollection::NodeCollection(MapArranger *parent):
        parent(parent)
{

}

void NodeCollection::addInstancesFromVector(std::vector<NodeInstance*>&& nodeDict)
{
    experiences = std::move(nodeDict);
    for (const auto node : experiences) {
        nodeSets[node->sizeOfExits()].insert(node);
    }
}
