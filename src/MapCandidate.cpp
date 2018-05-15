//
// Created by stumbo on 18-5-14.
//
#include <ostream>

#include "MapCandidate.h"
#include "NodeInstance.h"

/**
 * consider the new node instance as a new node in the map
 * @param instance the node instance
 * @param arriveAt the exit number arrive at
 */
void MapCandidate::createNewNode(NodeInstance *instance, uint8_t arriveAt) {
    auto nodeResult = nodes.emplace(new TopoNode(instance));
    if (!nodeResult.second) {
        cout << "createNewNode FAILURE!!!" << endl;
        throw;
    }
    fullEdgeNumber += instance->getExitNums();
    auto newNodePtr = *nodeResult.first;
    auto edgeResult = edges.emplace(new TopoEdge(currentNode, leaveFrom, newNodePtr, arriveAt));
    if (!edgeResult.second) {
        cout << "createNewNode FAILURE!!!" << endl;
        throw;
    }
    currentNode = newNodePtr;
}

void MapCandidate::arriveAtNode(NodeInstance *instance, uint8_t arriveAt) {
    if (currentEdge == nullptr) {
        createNewNode(instance, arriveAt);
        justArriveNew = true;
    } else {
        TopoNode * shouldToNode = currentEdge->getAnotherNode(currentNode);
        if (*instance == *shouldToNode->getCorrespondInstance()
            && arriveAt == currentEdge->getAnotherGate(currentNode)) {
            currentNode = shouldToNode;
            justArriveNew = false;
        } else {
            suicide();
        }
    }
}

/**
 * SPLIT A NEW CANDIDATE FROM A JUST ADDED NEW NODE CANDIDATE
 * @param arriveNode
 * @param arriveGate
 * @return if it is a similar return the new similar ptr, otherwise a nullptr
 */
MapCandidate* MapCandidate::arriveAtSimiliar(TopoNode *arriveNode, uint8_t arriveGate) {
    if (arriveNode->isExitEmpty(arriveGate)) {  //TODO 其他判定条件
        MapCandidate* newMap = new MapCandidate(*this); //TODO 创建一个新的map,需要完成拷贝赋值函数
    } else {
        return nullptr;
    }
}

/**
 * @param exit set the exit number to leave
 */
void MapCandidate::setLeaveFrom(uint8_t exit) {
    leaveFrom = exit;
    currentEdge = currentNode->getEdge(exit);
}


void MapCandidate::suicide() {
    for (auto iter: edges) {
        delete *iter;
    }
    for (auto iter: nodes) {
        delete *iter;
    }
    //TODO 调用析构本Map
}

TopoEdge::TopoEdge(const TopoEdge &) {

}
