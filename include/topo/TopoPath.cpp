//
// Created by stumbo on 19-10-23.
//

#include "TopoPath.h"

MapCandidate* TopoPath::setInvalid()
{
    auto res = relatedMap;
    relatedMap = nullptr;
    return res;
}

bool
TopoPath::findPath(MapCandidate* targetMap, NodeInstance* beginNode, NodeInstance* goalNode)
{
    targetNode = goalNode;
    relatedMap = targetMap;
    return true;
}
