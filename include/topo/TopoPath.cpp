//
// Created by stumbo on 19-10-23.
//

#include "TopoPath.h"
#include "Topo.h"
#include <map>
#include <vector>
#include <algorithm>

using namespace std;

/**
 * @brief set the topopath to be invalid, mostly caused by the purge of the related map
 * @return the removed map candidate
 */
MapCandidate* TopoPath::setInvalid()
{
    auto res = relatedMap;
    relatedMap = nullptr;
    return res;
}

/**
 * @brief find a path using A* algorithm, the answer would be stored in the TopoPath::path
 * @param targetMap the map where the algorithm runs on
 * @param beginIns the NodeInstance of the begin point
 * @param goalIns the NodeInstance of the goal point
 * @return wheather the path if successfully generated
 */
bool
TopoPath::findPath(MapCandidate* targetMap, NodeInstance* beginIns, NodeInstance* goalIns)
{
    /// this is the data needed to store in the TopoNode
    struct AStarHelper
    {
        /// the evaluated cost to arrive at goal
        double H = 0.0;
        /// the cost since the begin point
        double G = 0.0;
        TopoEdge* edge2Father = nullptr;
        bool closed = false;

        AStarHelper(TopoNode* master, double targetX, double targetY) {
            double masterX = master->getInsCorrespond()->getGlobalX();
            double masterY = master->getInsCorrespond()->getGlobalY();
            double dx = masterX - targetX;
            double dy = masterY - targetY;
            H = sqrt(dx * dx + dy * dy);
        }
    };

    /// a memory block to store the helpers
    vector<AStarHelper> aStarfreeList;
    aStarfreeList.reserve(targetMap->getNodeNum());
    /// clean the ptr slots in the TopoNodes (where the helpers are stored)
    targetMap->cleanAllNodeFlagsAndPtr();

    targetNode = goalIns;
    relatedMap = targetMap;
    TopoNode* const topoBegin = beginIns->getNodeUsageOfGivenMap(targetMap);
    TopoNode* const topoGoal = goalIns->getNodeUsageOfGivenMap(targetMap);
    double targetX = goalIns->getGlobalX();
    double targetY = goalIns->getGlobalY();

    /// the candidate list of A* algorithm piar of <G+H, TopoNode>
    map<double, vector<TopoNode*>> openList;

    aStarfreeList.emplace_back(topoBegin, targetX, targetY);
    auto firstHelper = &aStarfreeList.back();;
    topoBegin->setAssistPtr(firstHelper);
    openList[firstHelper->G + firstHelper->H].push_back(topoBegin);

    while(topoGoal->getAssistPtr() == nullptr)
    {
        /// the openList runs out, which means the goal is unreachable
        if (openList.empty()) {
            cerr << __FILE__ << ":" << __LINE__
                            << "[ERROR] a path planning is failure!\n"
                               "Not connected?!" << endl;
            return false;
        }

        /// pick out the best candidate TopoNode from the list
        auto & TopoNodeVecWithLowestF = openList.begin()->second;
        TopoNode* currentNode = TopoNodeVecWithLowestF.back();
        TopoNodeVecWithLowestF.pop_back();
        /// make sure all keys in the openList is not empty
        if (TopoNodeVecWithLowestF.empty()) {
            openList.erase(openList.begin());
        }
        auto baseHelper = static_cast<AStarHelper*>(currentNode->getAssistPtr());
        if (baseHelper->closed) {
            /// if the helper.G is changed durning the later progress, the new key would be
            /// inserted again, and the older one would be reduplicative then comes to here
            continue;
        } else {
            baseHelper->closed = true;
        }

        const auto & edges = currentNode->getEdgeConnected();
        for (const auto & edge : edges) {
            /// not built yet, ignore
            if (edge == nullptr) {
                continue;
            }
            TopoNode * anotherNode = edge->getAnotherNode(currentNode);
            auto helper = static_cast<AStarHelper*>(anotherNode->getAssistPtr());
            /// not touched, add it to openlist
            if (helper == nullptr) {
                aStarfreeList.emplace_back(anotherNode, targetX, targetY);
                helper = &aStarfreeList.back();
                anotherNode->setAssistPtr(helper);
                helper->edge2Father = edge;
                if (anotherNode == topoGoal) {
                    /// wow, we make it!
                    break;
                }
                helper->G = edge->getOdomLen() + baseHelper->G;
                openList[helper->G + helper->H].push_back(anotherNode);
                continue;
            }
            if (helper->closed) {
                /// well, this one is used
                continue;
            } else {
                /// let's see if mine route is better
                double newG = edge->getOdomLen() + baseHelper->G;
                if (helper->G < newG) {
                    /// nope, his is faster
                    continue;
                } else {
                    /// mine is better! replace it, I AM YOUR FATHER
                    helper->G = newG;
                    helper->edge2Father = edge;
                    openList[newG + helper->H].push_back(anotherNode);
                    continue;
                }
            }
        }
    }

    /// we found the path, copy it
    path.clear();
    TopoNode* currentNode = topoGoal;
    while (true)
    {
        /// from the current node to find the back node, and record the connected edge
        auto currentHelper = static_cast<AStarHelper*>(currentNode->getAssistPtr());
        TopoEdge* connectedEdge = currentHelper->edge2Father;
        currentNode = connectedEdge->getAnotherNode(currentNode);
        path.push_back({currentNode, connectedEdge});
        if (currentNode == topoBegin) {
            /// remember to reverse it to begin->goal order
            reverse(path.begin(), path.end());
            return true;
        }
    }
}
