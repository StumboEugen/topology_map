//
// Created by stumbo on 18-5-10.
//

#include "ExitInstance.h"
#include "TopoTools.h"


ExitInstance::ExitInstance(double posx, double posy, double dir)
        :outDir(dir),
         midPosX(posx),
         midPosY(posy),
         midRad(atan2(posx, posy) - piHalf)
{
}

bool ExitInstance::alike(const ExitInstance & rExit) const {
    auto & lExit = *this;
    double dirDif = abs(rExit.outDir - lExit.outDir);
    if (dirDif > dirTollerance()) {
        return false;
    }

    //do not need to consider the midRad, it is just used to sort
    double posDif = abs(rExit.midPosX - lExit.midPosX) + abs(rExit.midPosY - lExit.midPosY);
    return  posDif < posTollerance();   //just to kill warning, may not a good idea
}

Json::Value ExitInstance::toJS() {
    JSobj obj;
    obj["pos"].append(midPosX);
    obj["pos"].append(midPosY);
    obj["dir"] = outDir;
    return std::move(obj);
}
