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
 * collection of the NodeInstances
 * a member in MapArranger
 */
class NodeCollection {
public:
    explicit NodeCollection(MapArranger * parent);

    vector<MapCandidate *> addInstanceAndCompare(NodeInstance *newIns, uint8_t arriveAt,
                          double dis_x, double dis_y);

    const map<int, set<NodeInstance *>> &getNodeSets() const {
        return nodeSets;
    }

    size_t experienceSize() const {
        return experiences.size();
    }

    JSobj toJS() const;

    void clear();

    void addInstanceDirectly(NodeInstance * newNode);

private:
    MapArranger * const parent;

    std::map<int, std::set<NodeInstance*>> nodeSets;

    std::vector<NodeInstance*> experiences;
};
//TODO 更多的分类标准


#endif //TOPOLOGY_MAP_NODECOLLECTION_H
