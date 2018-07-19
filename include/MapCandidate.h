//
// Created by stumbo on 18-5-14.
//

#ifndef TOPOLOGY_MAP_MAPCANDIDATE_H
#define TOPOLOGY_MAP_MAPCANDIDATE_H

#include <memory>
#include <vector>
#include <set>
#include <list>
#include <ostream>
#include "TopoType.h"

using namespace std;

class TopoEdge;
class TopoNode;
class NodeInstance;

/**
 * represent a possibility of the real topo structure map
 * using TopoEdges and TopoNodes
 */
class MapCandidate {
public:
    explicit MapCandidate(NodeInstance *);
    MapCandidate(const MapCandidate&);
    void setLeaveFrom(gateId exit);
    bool arriveAtNode(NodeInstance * instance, gateId arriveAt, double dis_x, double dis_y);
    MapCandidate *const arriveAtSimiliar(TopoNode *arriveNode, gateId arriveGate);

    size_t getFullEdgeNumber() const {
        return fullEdgeNumber;
    }

    const unsigned long getNodeNum() const {
        return nodes.size();
    }

    const unsigned long getEdgeNum() const {
        return edges.size();
    }

    bool isJustMovedOnKnownEdge() const {
        return justMovedOnKnownEdge;
    }

    void setListPosition(std::list<MapCandidate *>::iterator listPos) {
        listPosition = listPos;
    }

    std::list<MapCandidate *>::iterator getListPosition() {
        return listPosition;
    }

    TopoNode *const addNewNode(NodeInstance * instance);

    TopoEdge *const addNewEdge(TopoNode * ea, gateId ga, TopoNode * eb, gateId gb);

    void removeNode(TopoNode * node2remove);

    ~MapCandidate();

private:
    void arriveNewNode(NodeInstance *instance, gateId arriveAt);
    set<TopoNode *> nodes;
    set<TopoEdge *> edges;
    TopoNode * lastNode;
    /**CAN BE NULL
     * nullptr means moving on an edge have never been to*/
    TopoEdge * lastEdge;
    bool justMovedOnKnownEdge;
    uint8_t leaveFrom;
    size_t fullEdgeNumber;

    std::list<MapCandidate *>::iterator listPosition;
};



#endif //TOPOLOGY_MAP_MAPCANDIDATE_H
