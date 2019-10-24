//
// Created by stumbo on 19-10-23.
//

#ifndef TOPOLOGY_MAP_TOPOPATH_H
#define TOPOLOGY_MAP_TOPOPATH_H

#include <vector>

class MapCandidate;
class NodeInstance;
class TopoNode;
class TopoEdge;

class TopoPath
{
    struct PathStep
    {
        TopoNode* beginNode;
        TopoEdge* stepEdge;
    };

public:

    bool findPath(
            MapCandidate * targetMap, NodeInstance * beginNode, NodeInstance * targetNode);

    MapCandidate* setInvalid();


    MapCandidate* getRelatedMap() const
    {
        return relatedMap;
    }

    NodeInstance *getTargetNode() const
    {
        return targetNode;
    }

    int getCurrentProgress() const
    {
        return currentProgress;
    }

    const std::vector<PathStep>& getPath() const
    {
        return path;
    }


private:
    /// the path related mapcandidate, would be NULLPTR if the map is wrong
    MapCandidate * relatedMap;

    /// the target nodeInstance
    NodeInstance * targetNode;

    /// on which step on the path
    int currentProgress;

    /// where is the agent on the path
    std::vector<PathStep> path;
};


#endif //TOPOLOGY_MAP_TOPOPATH_H
