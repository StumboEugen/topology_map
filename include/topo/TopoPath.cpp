//
// Created by stumbo on 19-10-23.
//

#include "TopoPath.h"
#include "Topo.h"
#include <map>
#include <vector>
#include <algorithm>

using namespace std;

/**
 * @brief 设置对应的路径无效化, 可能是因为对应的 MapCandidate 是错误构图, 也可能因为没有按照路线移动
 * @return 之前用来寻路的 MapCandidate 的指针
 */
MapCandidate* TopoPath::setInvalid()
{
    auto res = relatedMap;
    relatedMap = nullptr;
    return res;
}

/**
 * @brief 使用A*算法寻找路径, 结果会被保存在 TopoPath::path
 * @param targetMap 进行路径搜索的地图
 * @param beginIns 表示起点的 NodeInstance
 * @param goalIns 表示终点的 NodeInstance
 * @return 是否成功找到路径, 结果会被保存在 TopoPath::path
 */
bool
TopoPath::findPath(MapCandidate* targetMap, NodeInstance* beginIns, NodeInstance* goalIns)
{
    /// A* 辅助数据结构, 需要存储在 TopoNode 中
    struct AStarHelper
    {
        /// 估计的当前 TopoNode 距离终点的代价, 目前用的就是距离
        double H = 0.0;
        /// 目前距离起点已经付出的代价, 目前用的就是距离
        double G = 0.0;
        /// 从哪个 TopoNode 经过哪个 TopoEdge 来
        TopoEdge* edge2Father = nullptr;
        /// 是否已经被遍历过
        bool closed = false;

        AStarHelper(TopoNode* master, double targetX, double targetY) {
            double masterX = master->getInsCorrespond()->getGlobalX();
            double masterY = master->getInsCorrespond()->getGlobalY();
            double dx = masterX - targetX;
            double dy = masterY - targetY;
            H = sqrt(dx * dx + dy * dy);
        }
    };

    /// 一个整块申请的内存, 用于存储临时的 AStarHelper
    vector<AStarHelper> aStarfreeList;
    aStarfreeList.reserve(targetMap->getNodeNum());
    /// 清除所有 TopoNode 的辅助执政位, 用于存储 AStarHelper
    targetMap->cleanAllNodeFlagsAndPtr();

    goalIns = goalIns;
    relatedMap = targetMap;
    TopoNode* const topoBegin = beginIns->getNodeUsageOfGivenMap(targetMap);
    TopoNode* const topoGoal = goalIns->getNodeUsageOfGivenMap(targetMap);
    double targetX = goalIns->getGlobalX();
    double targetY = goalIns->getGlobalY();

    /// 按照G+H排序的候选 TopoNode 表, A* 算法的核心
    map<double, vector<TopoNode*>> openList;

    aStarfreeList.emplace_back(topoBegin, targetX, targetY);
    auto firstHelper = &aStarfreeList.back();;
    topoBegin->setAssistPtr(firstHelper);
    openList[firstHelper->G + firstHelper->H].push_back(topoBegin);

    while(topoGoal->getAssistPtr() == nullptr)
    {
        /// openList用完了, 这意味着没有找到通路, 目前的情况下这应该是不可能的
        if (openList.empty()) {
            cerr << __FILE__ << ":" << __LINE__
                            << "[ERROR] a path planning is failure!\n"
                               "Not connected?!" << endl;
            return false;
        }

        /// 挑选一个估计总代价较小的 TopoNode
        auto & TopoNodeVecWithLowestF = openList.begin()->second;
        TopoNode* currentNode = TopoNodeVecWithLowestF.back();
        TopoNodeVecWithLowestF.pop_back();
        /// 确保openList的每一个键值对都有值, 而不会出现空Vector
        if (TopoNodeVecWithLowestF.empty()) {
            openList.erase(openList.begin());
        }
        auto baseHelper = static_cast<AStarHelper*>(currentNode->getAssistPtr());
        if (baseHelper->closed) {
            /// 过程中如果helper.G减少了, 不会改变原来对应的键值对, 而是插入一个新的, 那么等到后来如果遇
            /// 到了这个老的键值对, 需要确保不会重复操作, 检查closed就可以做到
            continue;
        } else {
            baseHelper->closed = true;
        }

        /// 检查目标 TopoNode 的每一个相邻节点
        const auto & edges = currentNode->getEdgeConnected();
        for (const auto & edge : edges) {
            /// 对应的 TopoEdge 是空的(还没建图), 跳过
            if (edge == nullptr) {
                continue;
            }
            TopoNode * anotherNode = edge->getAnotherNode(currentNode);
            auto helper = static_cast<AStarHelper*>(anotherNode->getAssistPtr());
            /// 还没有碰到过, 建立对应的A*helper
            if (helper == nullptr) {
                aStarfreeList.emplace_back(anotherNode, targetX, targetY);
                helper = &aStarfreeList.back();
                anotherNode->setAssistPtr(helper);
                helper->edge2Father = edge;
                if (anotherNode == topoGoal) {
                    /// 这个就是终点! 找到了路径, 准备记录这个路径
                    break;
                }
                helper->G = edge->getOdomLen() + baseHelper->G;
                openList[helper->G + helper->H].push_back(anotherNode);
                continue;
            }
            if (helper->closed) {
                /// 这个TopoNode已经被遍历过了, 忽略
                continue;
            } else {
                /// 看一看这个备选的路径是不是不如从 current 走
                double newG = edge->getOdomLen() + baseHelper->G;
                if (helper->G < newG) {
                    /// 并没有, 还是别人的更快
                    continue;
                } else {
                    /// 从我这里走更快! 更新它的G, 并且设设置我为father
                    helper->G = newG;
                    helper->edge2Father = edge;
                    openList[newG + helper->H].push_back(anotherNode);
                    continue;
                }
            }
        }
    }

    /// 发现了路径, 复制到path中
    initPath();
    TopoNode* currentNode = topoGoal;
    while (true)
    {
        /// 从尾巴向头部开始遍历, 记录每一步的信息
        auto currentHelper = static_cast<AStarHelper*>(currentNode->getAssistPtr());
        TopoEdge* connectedEdge = currentHelper->edge2Father;
        currentNode = connectedEdge->getAnotherNode(currentNode);
        path.push_back({currentNode, connectedEdge});
        if (currentNode == topoBegin) {
            /// 最后将路径的内容反转, 使之获得正确的顺序
            reverse(path.begin(), path.end());
            return true;
        }
    }
}

/**
 * @brief 通知 TopoPath 向前移动一步
 * @return 是否成功向前移动, 失败可能: <br>
 * 1. 对应的 MapCandidate 被删除, 路径无效
 * 2. 路径已经走完
 * 3. 之前已经脱轨没有按照路径走
 * 4. 这次没有按照路径走
 */
bool TopoPath::stepForward()
{
    if (!hasSteps2Go()) {
        return false;
    }
    auto & lastStep = path[currentProgress];
    if (relatedMap->getCurrentNode() ==
        lastStep.stepEdge->getAnotherNode(lastStep.beginNode))
    {
        currentProgress++;
        return true;
    }
    notFollowingPath = true;
    return false;
}

/**
 * @brief 是否已经走完全程 (如果路径已经无效, 则为否)
 */
bool TopoPath::isFinished() const
{
    if (!isValid()) {
        return false;
    }
    return currentProgress == path.size();
}

/**
 * @brief 重置路径状态, 清空path, 进度清零, 脱轨标志位置true
 */
void TopoPath::initPath()
{
    path.clear();
    currentProgress = 0;
    notFollowingPath = false;
}

bool TopoPath::hasSteps2Go() const
{
    return isValid() && !isFinished() && !notFollowingPath;
}
