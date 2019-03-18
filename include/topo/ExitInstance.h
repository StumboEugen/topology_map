//
// Created by stumbo on 18-5-10.
//

#ifndef TOPOLOGY_MAP_NODEEXIT_H
#define TOPOLOGY_MAP_NODEEXIT_H

#include <memory>
#include <cmath>

#include "TopoTools.h"

using namespace std;

/**
 * represent an exit in a node instance
 */
class ExitInstance {
public:
    ExitInstance() = delete;

    explicit ExitInstance(double posx, double posy, double dir);

    /// the exit outward direction(NOTHING to do with the instance) ENU
    const double & getOutDir() const {
        return outDir;
    }

    /// relative coor to the mid of the instance, ENU
    const double & getPosX() const {
        return midPosX;
    }

    /// relative coor to the mid of the instance, ENU
    const double & getPosY() const {
        return midPosY;
    }

    /// the dir of the vector(mid to exit) ENU (NORTH = 0/360)
    const double & getMidRad() const {
        return midRad;
    }

    /**
     * sort according to the midRad(fron atan2) ENU
     */
    bool operator < (const ExitInstance & anotherNode) const {
        return this->midRad < anotherNode.midRad;
    }

    /**
     * determine if the two exits are alike
     */
    bool alike(const ExitInstance &) const;

    /**
     * the eixt outward dir tollerance
     */
    static double dirTollerance() {
        return 30.0;
    }

    /**
     * the pos tollerance
     */
    static double posTollerance() {
        return 0.5;
    }

    JSobj toJS() const;

private:
    // the exit outward direction(NOTHING to do with the instance) ENU
    double outDir;
    // relative coor to the mid of the instance, ENU
    double midPosX;
    // relative coor to the mid of the instance, ENU
    double midPosY;
    // the dir of the vector(mid to exit), ENU (NORTH = 0/360)
    double midRad;
};


#endif //TOPOLOGY_MAP_NODEEXIT_H
