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
         fullEdgeNumber(copyFrom.fullEdgeNumber),
         confidence(copyFrom.confidence)
{
    /**
     * firstly copy all the TopoNode instances
     */
    for (auto oriNode: copyFrom.nodes) {
        TopoNode *clonedNodePtr = * this->nodes.emplace
                (new TopoNode(oriNode->getInsCorrespond())).first;
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
                                       oriEdge->getNodeA()->clonedTo,
                                       oriEdge->getNodeB()->clonedTo);
        this->edges.insert(clonedEdge);
        /**add the relationships in nodes;*/
        clonedEdge->registerAtNodes();

//        if (nodes.find(clonedEdge->getNodeA()) == nodes.end() ||
//            nodes.find(clonedEdge->getNodeB()) == nodes.end()) {
//            throw;
//        }
//
//        if (clonedEdge->getNodeA() == nullptr || clonedEdge->getNodeB() == nullptr) {
//            throw;
//        }

//        clonedEdge->getNodeA()->addEdge(clonedEdge->getGateA(), clonedEdge);
//        clonedEdge->getNodeB()->addEdge(clonedEdge->getGateB(), clonedEdge);
    }
    /**
     * other details
     */
     currentNode = copyFrom.currentNode->clonedTo;
     if (copyFrom.currentEdge != nullptr) {
         /**find the current edge from the node -> new node -> new edge*/
         currentEdge = copyFrom.currentEdge->getNodeA()->
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
                                const double dis_x, const double dis_y, const double yaw) {
    if (currentEdge == nullptr) {
        arriveNewNode(instance, arriveAt);
        // last Edge will not be null after arriveNewNode function
        currentEdge->addOdomData(dis_x, dis_y, yaw, currentEdge->getAnotherNode(currentNode));
        lastEdgeIsOldEdge = false;
    } else {
        TopoNode * shouldToNode = currentEdge->getAnotherNode(currentNode);
        /**if we used to move on this edge, the arrival situation should remain the same
         * otherwise it is a confilct, this candidate is wrong*/

        /// sth like 1, 359 may happen
        int diff = instance->alike(*shouldToNode->getInsCorrespond());
        arriveAt += diff;
        arriveAt %= instance->sizeOfExits();
        if (diff != -1 && arriveAt == currentEdge->getAnotherGate(currentNode)) {
            currentEdge->leaveFromNode(currentNode);
            confidence *= currentEdge->addOdomData(dis_x, dis_y, yaw, currentNode);
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
    //TODO move this outwards
    /**you may match a node on a map, but just moved on a known edge, in this case, return*/
    if (!arriveNode->isExitEmpty(arriveGate)) {
        /**because it is called for a new candidate, so we don't let @this suicide()*/
        return nullptr;
    }

    if (arriveNode == currentEdge->getAnotherNode(currentNode)) {
        /// we don't consider the case where the node can conncet to itself
        return nullptr;
    }

    auto newMap = new MapCandidate(*this);
    /**@attention change is applied on the new map, not "this" */

    /// the node that happens loop close
    auto loopClosureNode = arriveNode->clonedTo;

    /// in the new map, the edge should connect form the last node to the loop-closed node
    newMap->currentEdge->changeExitTo(newMap->currentNode, loopClosureNode, arriveGate);

    /// record the newset ins, because we are going to kill and unregister the old cur node
    auto theLatestIns = newMap->currentNode->getTheLastRelatedIns();

    /// remove the node and unregister
    newMap->removeNode(newMap->currentNode);

    /// remove the last useage in the loop-closed node related ins
    loopClosureNode->getTheLastRelatedIns()->removeUseage(this);

    /// add the useage in the latest ins
    theLatestIns->addUseage(this, loopClosureNode);
    loopClosureNode->addNewRelatedIns(theLatestIns);

    newMap->currentNode = loopClosureNode;

//    auto & nodes = newMap->getNodes();
//    auto & edges = newMap->getEdges();
//    for (auto & edge : edges) {
//        if (nodes.find(edge->getNodeA()) == nodes.end() ||
//            nodes.find(edge->getNodeA()) == nodes.end()) {
//            throw;
//        }
//    }

    return newMap;
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
TopoNode *const MapCandidate::addNewNode(NodeInstance *const instance) {
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
 * @warning didn't check if the TopoNodes belongs to this Mapcandidate
 * @return the ptr of the edge
 */
TopoEdge *const
MapCandidate::addNewEdge(TopoNode *const ea, gateId ga, TopoNode *const eb, gateId gb) {
    //TODO cancel this check
    if (nodes.find(ea) == nodes.end() || nodes.find(eb) == nodes.end()) {
        cerr << "[MapCandidate::addNewEdge]"
                "you try to connect two node that didn't belong to this mapcandidate!"
                << endl;
    }

    auto edgeResult = edges.emplace(new TopoEdge(ea, ga, eb, gb));
    if (!edgeResult.second) {
        cout << "[MapCandidate::addNewEdge] FAILURE!!!" << endl;
        throw;
    }
    auto newEdge = (* edgeResult.first);
    newEdge->registerAtNodes();
//    ea->addEdge(ga, newEdge);
//    eb->addEdge(gb, newEdge);
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
    obj["edges"] = Json::nullValue;
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
    obj["confidence"] = confidence;
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
    /// the built Topo Node is stored here
    std::vector<TopoNode*> nodeDict{nodeInses.size(), nullptr};
    auto & JSedges = JSinfo["edges"];
    if (JSedges.isNull()) {
        const auto & theOnlyNode = addNewNode(nodeInses[0]);
        nodeDict[0] = theOnlyNode;
    }
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

        tempEdgePtr->addOdomData(JSedge["Ox"].asDouble(),
                                 JSedge["Oy"].asDouble(),
                                 JSedge["yaw"].asDouble(),
                                 nodeDict[serial_A]);
        //TODO add directly

        if (JSedge.isMember("cur")) {
            this->currentEdge = tempEdgePtr;
        }
    }

    this->confidence = JSinfo["confidence"].asDouble();
    this->currentNode = nodeDict[JSinfo["curNode"].asUInt()];
    this->lastEdgeIsOldEdge = JSinfo["lEiOe"].asBool();

    // using this as a way of checking
    if (JSinfo["edgeFullNum"].asUInt() != this->fullEdgeNumber) {
        std::cerr << "[ERROR] JS to Map errpr, full edge number not equal!" << endl;
    }
}

void MapCandidate::cleanAllNodeFlagsAndPtr() {
    for (const auto & node : nodes) {
        node->cleanFlags();
        node->setAssistPtr(nullptr);
    }
}

void MapCandidate::xConfidence(double coe) {
    confidence *= coe;
}

double MapCandidate::getConfidence(double experienceCountK) const {
    double N = nodes.size();
    return confidence * exp(-N * log(experienceCountK));
}

void MapCandidate::detachAllInstances() {
    for (auto & node : nodes) {
        node->getInsCorrespond()->removeUseage(this);
    }
}

double MapCandidate::getConfidencePURE() const {
    return confidence;
}

