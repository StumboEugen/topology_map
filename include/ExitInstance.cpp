//
// Created by stumbo on 18-5-10.
//

#include "ExitInstance.h"


ExitInstance::ExitInstance(double posx, double posy, double dir)
        :outDir(dir),
         midPosX(posx),
         midPosY(posy),
         midAng(piHalf - atan2(posx, posy))
{}
