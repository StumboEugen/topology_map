//
// Created by stumbo on 18-5-21.
//
#include <iostream>

using namespace std;

#include "MapArranger.h"
#include "NodeInstance.h"

void MapArranger::arriveInstance(NodeInstance *instance, uint8_t arriveAt,
                                 double dis_x, double dis_y) {
    if (!instance->isAddComplete()) {
        cout << "[MapArranger::arriveInstance] You add a uncompleted node to the collection!" << endl;
    }
    /**
     * firstly apply this move to every map
     */
    mapCollect.arriveNodeInstance(instance, arriveAt, dis_x, dis_y);
    /**
     * secondly find the similiar ones
     */
    auto newMaps = nodeCollect.addInstanceAndCompare(instance, arriveAt, dis_x, dis_y);
    /**
     * add the similiar ones to the map collection
     */
    for (auto & newmap : newMaps) {
        mapCollect.addNewMap(newmap);
    }
}
