//
// Created by stumbo on 18-5-21.
//

#ifndef TOPOLOGY_MAP_NODECOLLECTION_H
#define TOPOLOGY_MAP_NODECOLLECTION_H

#include <set>
#include <map>
#include <unordered_map>

#include "TopoTools.h"

using namespace std;

class MapArranger;
/**
 * a container of NodeInstance s.
 * currently there are two container in this, vector contains the NodeInstance in a time order.
 * which means the experience of the environment.
 * \n
 * the second one is in std::map, for quick search.
 * for example, the 4 exits node is in the nodeSets[4]
 */
class NodeCollection {
public:
    // the main constructor.
    explicit NodeCollection(MapArranger * parent);

    // compare the instances with every node and return the maps happen loop-closure.
    vector<MapCandidate *> addInstanceAndCompare(NodeInstance *newIns, uint8_t arriveAt,
                          double dis_x, double dis_y);

    // convert the NodeCollection to JSON structure
    JSobj toJS() const;

    // clear all info and destroy NodeInstances.
    void clear();

    // manually add a NodeInstance.
    void addInstanceDirectly(NodeInstance * newNode);

    const map<int, set<NodeInstance *>> &getNodeSets() const {
        return nodeSets;
    }

    size_t experienceSize() const {
        return experiences.size();
    }

private:

    /// the parent MapArranger
    MapArranger * const parent;

    /// the NodeInstance stored according to the features(exit number)
    std::map<int, std::set<NodeInstance*>> nodeSets;

    /// the NodeInstance stored in the time order
    std::vector<NodeInstance*> experiences;
};


#endif //TOPOLOGY_MAP_NODECOLLECTION_H
