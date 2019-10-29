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

/**
 * @brief 用于表征在已经建好的 MapCandidate 中的寻路, 包含A*算法 ,存储在 MapCollection::currentPath
 */
class TopoPath
{
    /// 表征每一步对应的 TopoNode 以及下一步要走的 TopoEdge
    struct PathStep
    {
        /// 每一步的起点 TopoNode
        TopoNode* beginNode;
        /// 每一步下一步对应的 TopoEdge
        TopoEdge* stepEdge;
    };

public:

    bool findPath(
            MapCandidate * targetMap, NodeInstance * beginIns, NodeInstance * goalIns);

    MapCandidate* setInvalid();


    MapCandidate* getRelatedMap() const
    {
        return relatedMap;
    }

    NodeInstance *getGoalInstance() const
    {
        return goalInstance;
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
    /// 路径规划对应的 MapCandidate, 如果为 nullptr, 说明对应地图是错误的
    /// @see setInvalid()
    MapCandidate * relatedMap;

    /// 终点对应的 NodeInstance
    /// @note 这样可以在原地图 TopoPath::relatedMap 错误后快速切换,虽然目前感觉这个行为意义其实不大
    NodeInstance * goalInstance;

    /// 记录当前路径走到了第几步
    int currentProgress;

    /// 对应的路径, 记录 PathStep 类型, 表征每一步对应的 TopoNode 以及下一步要走的 TopoEdge
    std::vector<PathStep> path;
};


#endif //TOPOLOGY_MAP_TOPOPATH_H
