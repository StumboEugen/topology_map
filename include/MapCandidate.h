//
// Created by stumbo on 18-5-14.
//

#ifndef TOPOLOGY_MAP_MAPCANDIDATE_H
#define TOPOLOGY_MAP_MAPCANDIDATE_H

#include <memory>
#include <vector>
#include <set>
#include <ostream>

using namespace std;

class TopoEdge;
class TopoNode;
class NodeInstance;

class MapCandidate {
public:
    explicit MapCandidate(NodeInstance *);
    MapCandidate(const MapCandidate&);
    void setLeaveFrom(uint8_t exit);
    bool arriveAtNode(NodeInstance * instance, uint8_t arriveAt, double dis_x, double dis_y);
    MapCandidate *const arriveAtSimiliar(TopoNode *arriveNode, uint8_t arriveGate);

    const unsigned getFullEdigeNumber() const {
        return fullEdgeNumber;
    }

    const unsigned long getNodeNum() const {
        return nodes.size();
    }

    const unsigned long getEdgeNum() const {
        return edges.size();
    }

    bool isJustMovedOnKnownEdge() {
        return justMovedOnKnownEdge;
    }

    TopoNode *const addNewNode(NodeInstance * instance);

    TopoEdge *const addNewEdge(TopoNode * ea, uint8_t ga, TopoNode * eb, uint8_t gb);

    void removeNode(TopoNode * node2remove);

    ~MapCandidate();

private:
    void arriveNewNode(NodeInstance *instance, uint8_t arriveAt);
    set<TopoNode *> nodes;
    set<TopoEdge *> edges;
    TopoNode * lastNode;
    /**CAN BE NULL
     * means moving on an edge have never been to*/
    TopoEdge * lastEdge;
    bool justMovedOnKnownEdge;
    uint8_t leaveFrom;
    unsigned fullEdgeNumber;
};



#endif //TOPOLOGY_MAP_MAPCANDIDATE_H
