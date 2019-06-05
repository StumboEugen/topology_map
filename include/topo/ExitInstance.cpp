//
// Created by stumbo on 18-5-10.
//

#include "ExitInstance.h"
#include "TopoTools.h"

/// the only constructure of ExitInstance
/**
 * given the pos relative to the middle point of NodeInstance and the outwards direction
 * @param posx the position X relative to the middle point
 * @param posy the position Y relative to the middle point
 * @param dir the outwards of the exit, for example, the door is at (1,1) but the outward is
 * pointing at (0,1) north, it is different if it is pointing at (1,0) east
 * @attention the direction has no relationship with ExitInstance::midRad
 */
ExitInstance::ExitInstance(double posx, double posy, double dir)
        :outDir(dir),
         midPosX(posx),
         midPosY(posy)
{
    midRad = atan2(posy, posx);
}

/// compare another ExitInstance if they are alike
/**
 * @param rExit another ExitInstance
 * @return if they are alike
 * @attention it doesn't consider the midRad and the posDif is in 1-norm (一范数)
 */
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

/// convert ExitInstance into a JSON structure
/**
 * @return
 * ["pos"] array [posx, posy] \n
 * ["dir"] float ourwards \n
 */
JSobj ExitInstance::toJS() const {
    JSobj obj;
    obj["pos"].append(midPosX);
    obj["pos"].append(midPosY);
    obj["dir"] = outDir;
    return std::move(obj);
}

/// sort according to the midRad(from atan2) ENU
/**
 *  @see ExitInstance(double, double, double)
 */
bool ExitInstance::operator<(const ExitInstance & anotherNode) const {
    return this->midRad < anotherNode.midRad;
}
