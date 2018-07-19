//
// Created by stumbo on 18-5-10.
//

#include "ExitInstance.h"
#include "TopoType.h"


ExitInstance::ExitInstance(double posx, double posy, double dir)
        :outDir(dir),
         midPosX(posx),
         midPosY(posy),
         midRad(atan2(posx, posy) - piHalf)
{
    // TODO coor problem
//    if (midRad > pi - 0.2) {
//        midRad = -pi;
//    }
}

bool ExitInstance::alike(const ExitInstance & rExit) const {
    auto & lExit = *this;
    double dirDif = abs(rExit.outDir - lExit.outDir);
    if (dirDif > dirTollerance()) {
        return false;
    }

    double posDif = abs(rExit.midPosX - lExit.midPosX) + abs(rExit.midPosY - lExit.midPosY);
    return  posDif < posTollerance();   //just to kill warning, may not a good idea
}
