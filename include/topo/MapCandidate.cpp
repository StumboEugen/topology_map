//
// Created by stumbo on 18-5-14.
//
#include <ostream>

#include "MapCandidate.h"
#include "TopoNode.h"
#include "NodeInstance.h"
#include "TopoEdge.h"


/**
 * usually called at the very beginning
 * @param firstInstance the first node instance
 */
MapCandidate::MapCandidate( NodeInstance *const firstInstance):
        lastEdgeIsOldEdge(true),
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
         lastEdgeIsOldEdge(copyFrom.lastEdgeIsOldEdge),
         fullEdgeNumber(copyFrom.fullEdgeNumber)
{
    /**
     * firstly copy all the TopoNode instances
     */
    for (auto oriNode: copyFrom.nodes) {
        TopoNode *clonedNodePtr = * this->nodes.emplace(new TopoNode(oriNode->getInsCorrespond())).first;
        /**the useage should be added oustide of the constructor
         * because there is no map info in constructor
         */
        clonedNodePtr->getInsCorrespond()->addUseage(this, clonedNodePtr);
        oriNode->clonedTo = clonedNodePtr;
    }
    /**
     * secondly copy all the TopoEdge instances
     */
    for (auto oriEdge: copyFrom.edges) {
        auto clonedEdge = new TopoEdge(*oriEdge,
                                       oriEdge->getExitA()->clonedTo,
                                       oriEdge->getExitB()->clonedTo);
        this->edges.insert(clonedEdge);
        /**add the relationships in nodes;*/
        clonedEdge->getExitA()->addEdge(clonedEdge->getGateA(), clonedEdge);
        clonedEdge->getExitB()->addEdge(clonedEdge->getGateB(), clonedEdge);
    }
    /**
     * other details
     */
     currentNode = copyFrom.currentNode->clonedTo;
     if (copyFrom.currentEdge != nullptr) {
         /**find the current edge from the node -> new node -> new edge*/
         currentEdge = copyFrom.currentEdge->getExitA()->
                 clonedTo->getEdge(copyFrom.currentEdge->getGateA());
     } else {
         currentEdge = nullptr;
     }
}

/**
 * consider the new node instance as a new node in the map
 * @param instance the node instance
 * @param arriveAt the exit number arrive at
 */
inline void MapCandidate::arriveNewNode(NodeInstance *const instance, gateId arriveAt) {
    auto newNodePtr = addNewNode(instance);
    auto newEdgePtr = addNewEdge(currentNode, leaveFrom, newNodePtr, arriveAt);
    currentNode = newNodePtr;
    currentEdge = newEdgePtr;
}

/**
 * firstly called when a arrive action happened
 * @param instance the node instance
 * @param arriveAt the arrive gate
 * @return if the arrivial is OK for this map
 */
bool MapCandidate::arriveAtNode(NodeInstance *const instance, gateId arriveAt,
                                const double dis_x, const double dis_y) {
    if (currentEdge == nullptr) {
        arriveNewNode(instance, arriveAt);
        // last Edge will not be null after arriveNewNode function
        currentEdge->addOdomData(dis_x, dis_y, currentEdge->getAnotherNode(currentNode));
        lastEdgeIsOldEdge = false;
    } else {
        TopoNode * shouldToNode = currentEdge->getAnotherNode(currentNode);
        /**if we used to move on this edge, the arrival situation should remain the same
         * otherwise it is a confilct, this candidate is wrong*/
        if (instance->alike(*shouldToNode->getInsCorrespond())
            && arriveAt == currentEdge->getAnotherGate(currentNode)) {
            currentEdge->leaveFromNode(currentNode);
            currentEdge->addOdomData(dis_x, dis_y, currentNode);
            currentNode = shouldToNode;
            lastEdgeIsOldEdge = true;
        } else {
            return false;
        }
    }
    return true;
}

/**
 * SPLIT A NEW CANDIDATE FROM A JUST ADDED NEW NODE CANDIDATE.
 * in the new candidate, we should really arrive at the (arriveNode) at gate (arriveGate)
 * @param arriveNode
 * @param arriveGate
 * @return if it is a similar return the new similar ptr, otherwise a nullptr
 */
MapCandidate *const MapCandidate::arriveAtSimiliar(TopoNode *arriveNode, gateId arriveGate) {
    //TODO 其他判定条件
    /**you may match a node on a map, but just moved on a known edge, in this case, return*/
    if (!arriveNode->isExitEmpty(arriveGate)) {
        /**because it is called for a new candidate, so we don't let @this suicide()*/
        return nullptr;
    } else {
        auto newMap = new MapCandidate(*this);
        /**@attention change is applied on the new map, not "this" */
        newMap->currentEdge->changeExitTo(newMap->currentNode, arriveNode->clonedTo, arriveGate);
        //the old new node is replaced by the similiar node
        newMap->removeNode(newMap->currentNode);
        newMap->currentNode = arriveNode->clonedTo;
        return newMap;
    }
}

//TODO we may need a fuction to move according to pos & dir, not the id
/**
 * be called after the real leave action
 * @param exit set the exit number to leave
 */
void MapCandidate::setLeaveFrom(gateId exit) {
    leaveFrom = exit;
    currentEdge = currentNode->getEdge(exit);   //current Edge might be null
}

/**
 * add a new node in the set
 * @param instance
 * @return the ptr of the node
 */
inline TopoNode *const MapCandidate::addNewNode(NodeInstance *const instance) {
    auto nodeResult = nodes.emplace(new TopoNode(instance));
    if (!nodeResult.second) {
        cout << "[MapCandidate::addNewNode] FAILURE!!!" << endl;
        throw;
    }
    auto newNodePtr = *nodeResult.first;
    instance->addUseage(this, newNodePtr);
    fullEdgeNumber += instance->sizeOfExits();
    return newNodePtr;
}

/**
 * add a new edge in the set
 * @param instance
 * @return the ptr of the edge
 */
inline TopoEdge *const MapCandidate::addNewEdge(TopoNode *const ea, gateId ga, TopoNode *const eb, gateId gb) {
    auto edgeResult = edges.emplace(new TopoEdge(ea, ga, eb, gb));
    if (!edgeResult.second) {
        cout << "[MapCandidate::addNewEdge] FAILURE!!!" << endl;
        throw;
    }
    auto newEdge = (* edgeResult.first);
    ea->addEdge(ga, newEdge);
    eb->addEdge(gb, newEdge);
    return newEdge;
}

void MapCandidate::removeNode(TopoNode *node2remove) {
    fullEdgeNumber -= node2remove->getInsCorrespond()->sizeOfExits();
    node2remove->getInsCorrespond()->removeUseage(this);
    delete node2remove;
    nodes.erase(node2remove);
}

MapCandidate::~MapCandidate() {
    for (auto edge: edges) {
        delete edge;
    }
    for (auto node: nodes) {
        delete node;
    }
}

JSobj MapCandidate::toJS() const {
    JSobj obj;
    for (const auto & edge: edges) {
        auto edgeJS = edge->toJS();
        if (edge == currentEdge) {
            edgeJS["cur"] = true;
        }
        obj["edges"].append(edgeJS);
    }
    obj["curNode"] = currentNode->getInsCorrespond()->getSerialNumber();
    obj["lEiOe"] = lastEdgeIsOldEdge; //TODO is this needed?
    obj["edgeFullNum"] = fullEdgeNumber;
    return std::move(obj);
}

/**
 * call this function when this map has conflict, and need to be deleted
 */
void MapCandidate::removeUseages() {
    for (auto node: nodes) {
        node->getInsCorrespond()->removeUseage(this);
    }
}

/**
 * generate a candidate from the JS info
 * @param nodeInses the serialNum-nodeInstance pair container
 * @param JSinfo the JSinfo
 */
MapCandidate::MapCandidate(const std::vector<NodeInstance *> & nodeInses,
                           const JSobj & JSinfo) : fullEdgeNumber(0)
{
    std::vector<TopoNode*> nodeDict{nodeInses.size(), nullptr};
    auto & JSedges = JSinfo["edges"];
    for (int i = 0; i < JSedges.size(); i++) {
        auto & JSedge = JSedges[i];

        size_t serial_A = JSedge["Ea"].asUInt();
        if (nodeDict[serial_A] == nullptr) {
            nodeDict[serial_A] = addNewNode(nodeInses[serial_A]);
        }

        size_t serial_B = JSedge["Eb"].asUInt();
        if (nodeDict[serial_B] == nullptr) {
            nodeDict[serial_B] = addNewNode(nodeInses[serial_B]);
        }

        auto tempEdgePtr = addNewEdge(
                nodeDict[serial_A], static_cast<gateId>(JSedge["Ga"].asUInt()),
                nodeDict[serial_B], static_cast<gateId>(JSedge["Gb"].asUInt()));

        tempEdgePtr->addOdomData(
                JSedge["Ox"].asDouble(), JSedge["Oy"].asDouble(), nodeDict[serial_A]);
        //TODO add directly

        if (JSedge.isMember("cur")) {
            this->currentEdge = tempEdgePtr;
        }
    }

    this->currentNode = nodeDict[JSinfo["curNode"].asUInt() ];
    this->lastEdgeIsOldEdge = JSinfo["lEiOe"].asBool();

    // using this as a way of checking
    if (JSinfo["edgeFullNum"].asUInt() != this->fullEdgeNumber) {
        std::cerr << "[ERROR] JS to Map errpr, full edge number not equal!" << endl;
    }
}
