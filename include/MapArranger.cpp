//
// Created by stumbo on 18-5-21.
//
#include <iostream>

using namespace std;

#include "TopoMap.h"
#include "MapArranger.h"

void MapArranger::arriveInstance(NodeInstance *instance, gateId arriveAt,
                                 double dis_x, double dis_y) {
    if (!instance->isAddComplete()) {
        cout << "[MapArranger::arriveInstance] You add a uncompleted node to the collection!" << endl;
    }
    /**
     * firstly apply this move to every map
     */
    mapCollection.arriveNodeInstance(instance, arriveAt, dis_x, dis_y);
    /**
     * secondly find the similiar ones
     */
    auto newMaps = nodeCollection.addInstanceAndCompare(instance, arriveAt, dis_x, dis_y);
    /**
     * add the similiar ones to the map collection
     */
    for (auto & newmap : newMaps) {
        mapCollection.addNewMap(newmap);
    }
}

void MapArranger::moveThroughGate(gateId exit) {
    mapCollection.everyMapThroughGate(exit);
}

size_t MapArranger::getMapNumbers() {
    return mapCollection.mapNumbers();
}
