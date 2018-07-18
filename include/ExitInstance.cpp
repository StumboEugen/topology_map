//
// Created by stumbo on 18-5-10.
//

#include "ExitInstance.h"
#include "TopoType.h"


ExitInstance::ExitInstance(double posx, double posy, double dir)
        :outDir(dir),
         midPosX(posx),
         midPosY(posy),
         midAng(atan2(posx, posy) - piHalf)
{
    // TODO coor problem
    if (midAng > pi - 0.2) {
        midAng = -pi;
    }
}
