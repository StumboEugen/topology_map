//
// Created by stumbo on 18-5-14.
//
#include <ostream>


#include "MapCandidate.h"
#include "NodeInstance.h"

/**
 * usually called at the very beginning
 * @param firstInstance the first node instance
 */
MapCandidate::MapCandidate(const NodeInstance *const firstInstance):
        justArriveNew(true),
        leaveFrom(0),
        fullEdgeNumber(0),
        currentEdge(nullptr)
{
    auto firstNode = addNewNode(firstInstance);
    currentNode = firstNode;
}

/**
 * to copy a brand new map
 * @param copyFrom
 */
MapCandidate::MapCandidate(const MapCandidate & copyFrom)
        :leaveFrom(copyFrom.leaveFrom),
         justArriveNew(copyFrom.justArriveNew),
         fullEdgeNumber(copyFrom.fullEdgeNumber)
{
    /**
     * firstly copy all the TopoNode instances
     */
    for (auto oriNode: copyFrom.nodes) {
        TopoNode *clonedNodePtr = * this->nodes.emplace(new TopoNode(oriNode->getInstance())).first;
        oriNode->clonedTo = clonedNodePtr;
    }
    /**
     * secondly copy all the TopoEdge instances
     */
    for (auto oriEdge: copyFrom.edges) {
        TopoEdge *clonedEdge = new TopoEdge(oriEdge->getExitA()->clonedTo, oriEdge->getGateA(),
                                            oriEdge->getExitB()->clonedTo, oriEdge->getGateB());
        this->edges.insert(clonedEdge);
        /**add the relationships in nodes;*/
        clonedEdge->getExitA()->addEdge(clonedEdge->getGateA(), clonedEdge);
        clonedEdge->getExitB()->addEdge(clonedEdge->getGateB(), clonedEdge);
    }
    /**
     * other details
     */
     currentNode = copyFrom.currentNode->clonedTo;
     if (copyFrom.currentEdge != nullptr) {//TODO is the IF ELSE necessary?
         /**find the current edge from the node -> new node -> new edge*/
         currentEdge = copyFrom.currentEdge->getExitA()->clonedTo->getEdge(copyFrom.currentEdge->getGateA());
     } else {
         currentEdge = nullptr;
     }
}

/**
 * consider the new node instance as a new node in the map
 * @param instance the node instance
 * @param arriveAt the exit number arrive at
 */
void MapCandidate::arriveNewNode(const NodeInstance *const instance, uint8_t arriveAt) {
    auto newNodePtr = addNewNode(instance);
    auto newEdgePtr = addNewEdge(currentNode, leaveFrom, newNodePtr, arriveAt);
    currentNode = newNodePtr;
    currentEdge = newEdgePtr;
}

/**
 * called firstly when a arrive action happened
 * @param instance the node instance
 * @param arriveAt the arrive gate
 */
void MapCandidate::arriveAtNode(const NodeInstance *const instance, uint8_t arriveAt) {
    if (currentEdge == nullptr) {
        arriveNewNode(instance, arriveAt);
        justArriveNew = true;
    } else {
        TopoNode * shouldToNode = currentEdge->getAnotherNode(currentNode);
        /**if we used to move on this edge, the arrival situation should remain the same
         * otherwise it is a confilect, this candidate is wrong*/
        if (*instance == *shouldToNode->getInstance()
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
MapCandidate *const MapCandidate::arriveAtSimiliar(TopoNode *arriveNode, uint8_t arriveGate) {
    //TODO 其他判定条件
    if (arriveNode->isExitEmpty(arriveGate)) {
        auto newMap = new MapCandidate(*this);
        /**@attention change is applied on the new map, not "this" */
        newMap->currentEdge->changeExitTo(newMap->currentNode, arriveNode->clonedTo, arriveGate);
        newMap->removeNode(currentNode);
        newMap->currentNode = arriveNode->clonedTo;
    } else {
        return nullptr; /**because it is called for a new candidate, so we don't let @this suicide()*/
    }
}

/**
 * be called after the real leave action
 * @param exit set the exit number to leave
 */
void MapCandidate::setLeaveFrom(uint8_t exit) {
    leaveFrom = exit;
    currentEdge = currentNode->getEdge(exit);
}


void MapCandidate::suicide() {
    for (auto edge: edges) {
        delete edge;
    }
    for (auto node: nodes) {
        delete node;
    }
    //TODO 调用析构本Map
}

inline TopoNode *const MapCandidate::addNewNode(const NodeInstance *const instance) {
    auto nodeResult = nodes.emplace(new TopoNode(instance));
    if (!nodeResult.second) {
        cout << "addNewNode FAILURE!!!" << endl;
        throw;
    }
    fullEdgeNumber += instance->getExitNums();
    return * nodeResult.first;
}

inline TopoEdge *const MapCandidate::addNewEdge(TopoNode *const ea, uint8_t ga, TopoNode *const eb, uint8_t gb) {
    auto edgeResult = edges.emplace(new TopoEdge(ea, ga, eb, gb));
    if (!edgeResult.second) {
        cout << "addNewEdge FAILURE!!!" << endl;
        throw;
    }
    return * edgeResult.first;
}

/**
 * change one exit to another exit
 * @attention the node related will be modified too
 * @param oldNode
 * @param newNode
 * @param newGate
 */
void TopoEdge::changeExitTo(TopoNode *const oldNode, TopoNode *const newNode, const uint8_t newGate) {
    if (oldNode == exitB) {
        exitB->disconnectEdge(gateB);
        exitB = newNode;
        gateB = newGate;
        exitB->addEdge(gateB, this);
    } else if (oldNode == exitA) {
        exitA->disconnectEdge(gateA);
        exitA = newNode;
        gateA = newGate;
        exitA->addEdge(gateA, this);
    } else {
        cout << "TopoEdge changeExitTo FAILURE" << endl;
        throw;
    }
}
