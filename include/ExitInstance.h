//
// Created by stumbo on 18-5-10.
//

#ifndef TOPOLOGY_MAP_NODEEXIT_H
#define TOPOLOGY_MAP_NODEEXIT_H

#include <memory>
#include <cmath>

class NodeInstance;

using namespace std;

static const double piHalf = 3.1415926 / 2.0;

class ExitInstance {
public:
    ExitInstance() = delete;

    explicit ExitInstance(double posx, double posy, double dir);


    const double & Dir() const {
        return outDir;
    }

    const double & getPosX() const {
        return midPosX;
    }

    const double & getPosY() const {
        return midPosY;
    }

    const double & MidAng() const {
        return midAng;
    }

    /**
     * 按照atan2的结果进行排序
     */
    bool operator < (const ExitInstance & anotherNode) const {
        return this->midAng < anotherNode.midAng;
    }

private:
    double outDir;
    double midPosX;
    double midPosY;
    double midAng;
};


#endif //TOPOLOGY_MAP_NODEEXIT_H
