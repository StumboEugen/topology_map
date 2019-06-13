//
// Created by stumbo on 18-5-14.
//
#include <ostream>

#include "MapCandidate.h"
#include "TopoNode.h"
#include "NodeInstance.h"
#include "TopoEdge.h"


/**
 * @brief generate a MapCandidate that only has a TopoNode
 * usually called at the very beginning, no edge has been moved through.
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
 * @brief copy constructor
 * copy a MapCandidate, includes new TopoNode s and TopoEdge s, the new usages resulted from
 * the new MapCandidate would be refreshed too
 * @param copyFrom the MapCandidate copy source
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
        //todo
        TopoNode *clonedNodePtr = * this->nodes.emplace
                (new TopoNode(oriNode)).first;
        /**the useage should be added oustide of the TopoNode's constructor
         * because there is no map info in TopoNode's constructor
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
//        clonedEdge->registerAtNodes();

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
 * @brief inform the MapCandidate the robot arrives at a NodeInstance
 * @param instance the node instance
 * @param arriveAt the arrive gate
 * @return if the arrivial is OK for this map
 *
 * firstly called when a arrival action happened. In this function, this map would be deduced
 * to check if the new arrival is appropriate.
 *
 * For the MapCandidate that moves on a known TopoEdge, the new arrived NodeInstance would be
 * checked with the TopoNode that should to be.
 *
 * For the MapCandidate that moves on a unknown TopoEdge, the newly arrived NodeInstance
 * would be considered as a new TopoNode.
 *
 * Then the loop-closure would be checked at NodeCollection::addInstanceAndCompare
 *
 * @see NodeCollection::addInstanceAndCompare
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
 * @brief clone another MapCandidate from the origin one with a loop-closure
 * SPLIT A NEW CANDIDATE FROM A JUST ADDED NEW NODE CANDIDATE.
 *
 * This is called from NodeInstance, when the newly arrived NodeInstance is similiar to a
 * known TopoNode in this MapCandidate. Then a cloned MapCandidate with loop-closure would be
 * made.
 *
 * @param arriveNode the node that may happen loop closeure
 * @param arriveGate the arriveGate No in the older instance
 * @param gateDiff new instance's gateID + gateDiff = older instance's gateID
 * @return if it can be a similar map return the new map's ptr, otherwise a nullptr
 */
MapCandidate *const MapCandidate::arriveAtSimiliar(
        TopoNode *arriveNode, gateId arriveGate, int gateDiff) {

    //TODO move this outwards
    /**you may match a node on a map, but the arrived gate is occupied(has been moved through),
     * in this case, return*/
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

    /// the node that happens loop close (in the new map)
    auto loopClosureNode = arriveNode->clonedTo;

    /// in the new map, the edge should connect form the last node to the loop-closed node
    newMap->currentEdge->changeExitTo(newMap->currentNode, loopClosureNode, arriveGate);

    /// the newly arrived instance
    auto theLatestIns = newMap->currentNode->getTheLastRelatedIns();

    /// remove the node and unregister the useage in instance
    newMap->removeNode(newMap->currentNode);

    /// remove the last useage in the loop-closed node related ins
    /// because we only uses the newest instance to detect loop closure
    loopClosureNode->getTheLastRelatedIns()->removeUseage(newMap);

    /// add the useage in the latest ins
    theLatestIns->addUseage(newMap, loopClosureNode);
    loopClosureNode->addNewRelatedIns(theLatestIns);

    newMap->currentNode = loopClosureNode;

    if (gateDiff != 0) {
        loopClosureNode->ringRotate(
                loopClosureNode->getInsCorrespond()->sizeOfExits()-gateDiff);
    }
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

/**
 * @brief set the robot's state in \b this MapCandidate to leave at exit
 * @param exit set the exit number to leave
 * @todo we may need a fuction to move according to pos & dir, not the id
 */
void MapCandidate::setLeaveFrom(gateId exit) {
    leaveFrom = exit;
    currentEdge = currentNode->getEdge(exit);   //current Edge might be null
}

/**
 * @brief create a new node into the nodes set
 * @param instance the NodeInstance of the new TopoNode
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
 * @brief create a new edge into the nodes set
 * @attention didn't check if the TopoNodes belongs to this Mapcandidate
 * @return the ptr of the edge
 */
TopoEdge *const
MapCandidate::addNewEdge(TopoNode *const ea, gateId ga, TopoNode *const eb, gateId gb) {
//    //TODO cancel this check
//    if (nodes.find(ea) == nodes.end() || nodes.find(eb) == nodes.end()) {
//        cerr << "[MapCandidate::addNewEdge]"
//                "you try to connect two node that didn't belong to this mapcandidate!"
//                << endl;
//    }

    auto edgeResult = edges.emplace(new TopoEdge(ea, ga, eb, gb));
    if (!edgeResult.second) {
        cout << "[MapCandidate::addNewEdge] FAILURE!!!" << endl;
        throw;
    }
    auto newEdge = (* edgeResult.first);
//    newEdge->registerAtNodes();
//    ea->addEdge(ga, newEdge);
//    eb->addEdge(gb, newEdge);
    return newEdge;
}

/**
 * @brief erase a TopoNode (destroy) in the nodes set
 * The ueage in the NodeInstance would be purged too.
 * @param node2remove the TopoNode to remove
 */
void MapCandidate::removeNode(TopoNode *node2remove) {
    fullEdgeNumber -= node2remove->getInsCorrespond()->sizeOfExits();
    node2remove->getInsCorrespond()->removeUseage(this);
    delete node2remove;
    nodes.erase(node2remove);
}

/**
 * @brief the destructor destroy all TopoEdge and TopoNode.
 */
MapCandidate::~MapCandidate() {
    for (auto edge: edges) {
        delete edge;
    }
    for (auto node: nodes) {
        delete node;
    }
}

/**
 * @brief convert MapCandidate to JSON structure
 * @return
 * ["edges"] null if there are no edges yet(when just created) \n
 *           array of TopoEdges \n
 * ["cur"] bool the MapCandidate::currentEdge's "cur" would be true \n
 * ["nodes"] array of TopoNodes \n
 * ["curNode"] int MapCandidate::currentNode's serial number \n
 * ["lEiOe"] bool if the last moved on edge is a old edge \n
 * ["edgeFullNum"] int MapCandidate::fullEdgeNumber no need any more
 * ["confidence"] float the confidence of this MapCandidate, but it is PURE
 * @see TopoEdge::toJS()
 * @see TopoNode::toJS()
 */
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
    
    for (const auto & node: nodes) {
        obj["nodes"].append(node->toJS());
    }
    
    obj["curNode"] = currentNode->getInsCorrespond()->getSerialNumber();
    obj["lEiOe"] = lastEdgeIsOldEdge; //TODO is this needed?
    obj["edgeFullNum"] = fullEdgeNumber;
    obj["confidence"] = confidence;
    return std::move(obj);
}

/**
 * @brief remove all usages of the TopoNode s in related NodeInstances
 * call this function when this map has conflict, and need to be deleted
 * @warning You must call this before delete map
 */
void MapCandidate::removeUseages() {
    for (auto node: nodes) {
        node->getInsCorrespond()->removeUseage(this);
    }
}

/**
 * @brief generate a MapCandidate from the JSON info
 * @param nodeInses the serialNum-nodeInstance pair container
 * @param JSinfo the JSinfo of a map
 * @see MapCandidate::toJS()
 *
 * All work is done in MapCandidate class.
 *
 * @note
 * There are two versions of JSON structure, the difference is at the TopoNode related
 * NodeInstances. TopoNode in the newer one has several NodeInsatnce s stored
 *
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
    
    /// new version(since 2019-5-26) and old version split here
    if (JSinfo.isMember("nodes")) {
        // new version
        const auto & JSnodes = JSinfo["nodes"];
        
        // firstly build the nodes
        for (int i = 0; i < JSnodes.size(); ++i) {
            /// it's a array of related inses in the order of time
            const auto & JSnode = JSnodes[i];
            auto newNode = addNewNode(nodeInses[JSnode[0].asUInt()]);
            for (int j = 1; j < JSnode.size(); ++j) {
                newNode->addNewRelatedIns(nodeInses[JSnode[j].asUInt()]);
            }
            const auto theLastRelatedSerial = (--JSnode.end())->asUInt();
            nodeDict[theLastRelatedSerial] = newNode;
            nodeInses[theLastRelatedSerial]->addUseage(this, newNode);
        }
        
        // then the edges
        for (int i = 0; i < JSedges.size(); ++i) {
            const auto & JSedge = JSedges[i];
            size_t serial_A = JSedge["Ea"].asUInt();
            size_t serial_B = JSedge["Eb"].asUInt();
            
            auto tempEdgePtr = addNewEdge(
                    nodeDict[serial_A], static_cast<gateId>(JSedge["Ga"].asUInt()),
                    nodeDict[serial_B], static_cast<gateId>(JSedge["Gb"].asUInt()));

            tempEdgePtr->addOdomData(JSedge["Ox"].asDouble(),
                                     JSedge["Oy"].asDouble(),
                                     JSedge["yaw"].asDouble(),
                                     nodeDict[serial_A]);

            if (JSedge.isMember("cur")) {
                this->currentEdge = tempEdgePtr;
            }
        }
    } else {
        // old version

        // construct the edges
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

            if (JSedge.isMember("cur")) {
                this->currentEdge = tempEdgePtr;
            }
        }
    }

    this->confidence = JSinfo["confidence"].asDouble();
    this->currentNode = nodeDict[JSinfo["curNode"].asUInt()];
    this->lastEdgeIsOldEdge = JSinfo["lEiOe"].asBool();

//    // using this as a way of checking
//    if (JSinfo["edgeFullNum"].asUInt() != this->fullEdgeNumber) {
//        std::cerr << "[ERROR] JS to Map errpr, full edge number not equal!" << endl;
//    }
}

/**
 * @brief clear all assistance flags and ptrs of the TopoNodes
 */
void MapCandidate::cleanAllNodeFlagsAndPtr() {
    for (const auto & node : nodes) {
        node->cleanFlags();
        node->setAssistPtr(nullptr);
    }
}

/**
 * @brief update the confidence
 * @param coe the coefficient to x
 */
void MapCandidate::xConfidence(double coe) {
    confidence *= coe;
}

/**
 * @brief calculate the confidence of this map (not normalized)
 * @param experienceCountK the experience count, from node collection
 */
double MapCandidate::getConfidence(double experienceCountK) const {
    double N = nodes.size();
    return confidence * exp(-N * log(experienceCountK));
}