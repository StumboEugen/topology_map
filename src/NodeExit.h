//
// Created by stumbo on 18-5-10.
//

#ifndef TOPOLOGY_MAP_NODEEXIT_H
#define TOPOLOGY_MAP_NODEEXIT_H

#include <memory>
#include <cmath>

class TopoNode;

using namespace std;

static const double piHalf = 3.1415926 / 2.0;

class NodeExit {
public:
    NodeExit() = delete;

    explicit NodeExit(double posx, double posy, double dir)
            :outDir(dir),
             midPosX(posx),
             midPosY(posy),
             midAng(piHalf - atan2(posx, posy))
    {}

    const double & Dir() const {
        return outDir;
    }

    const double getPosX() const {
        return midPosX;
    }

    const double getPosY() const {
        return midPosY;
    }

    double & MidAng() {
        return midAng;
    }

    shared_ptr<NodeExit> & LeadTo() {
        return leadTo;
    }

    shared_ptr<TopoNode> & ExitForm() {
        return exitFrom;
    }

    /**
     * 按照atan2的结果进行排序
     */
    bool operator < (const NodeExit & anotherNode) const {
        return this->midAng < anotherNode.midAng;
    }

private:
    double outDir;
    double midPosX;
    double midPosY;
    double midAng;
    shared_ptr<NodeExit> leadTo;
    shared_ptr<TopoNode> exitFrom;
};


#endif //TOPOLOGY_MAP_NODEEXIT_H
