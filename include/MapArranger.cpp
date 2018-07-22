//
// Created by stumbo on 18-5-21.
//
#include <iostream>

using namespace std;

#include "TopoMap.h"
#include "MapArranger.h"

void MapArranger::arriveInstance(NodeInstance *instance, gateId arriveAt,
                                 double odomX, double odomY) {
    if (!instance->isAddComplete()) {
        cout << "[MapArranger::arriveInstance] You add a uncompleted node to the collection!" << endl;
    }
    /**
     * firstly apply this move to every map
     */
    mapCollection.arriveNodeInstance(instance, arriveAt, odomX, odomY);
    /**
     * secondly find the similiar ones
     */
    auto newMaps = nodeCollection.addInstanceAndCompare(instance, arriveAt, odomX, odomY);

    for (const auto & newMap: newMaps) {
        auto newPos = mapCollection.addNewMap(newMap.first, newMap.second);
        newMap.second->setListPosition(newPos);
    }
    experiences ++;
}

void MapArranger::moveThroughGate(gateId exit) {
    mapCollection.everyMapThroughGate(exit);
}

size_t MapArranger::getMapNumbers() {
    return mapCollection.mapNumbers();
}
