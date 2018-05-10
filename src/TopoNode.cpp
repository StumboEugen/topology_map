//
// Created by stumbo on 18-5-10.
//

#include "TopoNode.h"
#include "NodeExit.h"

#include <iostream>

using std::cout
using std::endl;

int TopoNode::addExit(double dir) {
    if (dir >= 360.0 || dir < 0.0) {
        cout << "[WARNING] you add a direction out of range:" << (int)dir << endl;
        dir =- 360.0 * (int)(dir/360.0);
    }
    NodeExit newExit;
    newExit.Dir() = dir;
    exits.insert(newExit);
    exits_count ++;
    return exits_count;
}
