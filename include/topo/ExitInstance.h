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
 * represent an exit in a NodeInstance
 */
class ExitInstance {
public:
    ExitInstance() = delete;

    // the only constructure of ExitInstance
    explicit ExitInstance(double posx, double posy, double dir);

    // sort according to the midRad(from atan2) ENU
    bool operator < (const ExitInstance & anotherNode) const;

    // compare another ExitInstance if they are alike
    bool alike(const ExitInstance &) const;

    //convert ExitInstance into a JSON structure
    JSobj toJS() const;

    // the exit outward direction (be different with the mid rad) ENU
    const double & getOutDir() const {
        return outDir;
    }

    // relative coor to the mid of the instance, ENU
    const double & getPosX() const {
        return midPosX;
    }

    // relative coor to the mid of the instance, ENU
    const double & getPosY() const {
        return midPosY;
    }

    /// the dir of the vector(mid to exit) in rad ENU (from atan2)
    /// @note \b example \n
    /// south/ east / north / \b west / east = \n -halfPI / 0 / halfPI / PI / -halfPI
    const double & getMidRad() const {
        return midRad;
    }

private: // function

    //the eixt outward dir tollerance in degree
    static double dirTollerance() {
        return 30.0;
    }

    // the tollerance of the pos
    static double posTollerance() {
        return 0.3;
    }

private: // members
    /// the exit outward direction ENU
    /// @attention the direction has no relationship with ExitInstance::midRad \n
    /// for example, the door is at (1,1) but the outward is pointing at (0,1) north, it is
    /// different if it is pointing at (1,0) east
    double outDir;

    /// relative coor to the mid of the instance, ENU
    double midPosX;

    /// relative coor to the mid of the instance, ENU
    double midPosY;

    /// the dir of the vector(middle point to exit), ENU (from atan2)
    /// @note \b example \n
    /// south/ east / north / \b west / east = \n -halfPI / 0 / halfPI / PI / -halfPI
    double midRad;
};


#endif //TOPOLOGY_MAP_NODEEXIT_H
