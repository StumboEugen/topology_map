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
