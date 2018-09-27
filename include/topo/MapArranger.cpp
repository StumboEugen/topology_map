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
        newMap.second->setPosInList(newPos);
    }
    experiences ++;
}

void MapArranger::moveThroughGate(gateId exit) {
    mapCollection.everyMapThroughGate(exit);
}

size_t MapArranger::getMapNumbers() {
    return mapCollection.mapNumbers();
}

MapArranger::MapArranger()
        : mapName(topo::getCurrentTimeString())
{}

JSobj MapArranger::toJS() const {
    JSobj obj;
    obj["nodeInstance"] = nodeCollection.toJS();
    obj["mapInstance"] = mapCollection.toJS();
    obj["Name"] = mapName;
    return std::move(obj);
}

using topo::checkJSMember;

bool MapArranger::readFromJSON(const JSobj &obj) {
    mapCollection.clear();
    nodeCollection.clear();

    if (!checkJSMember({"Name", "nodeInstance", "mapInstance"},obj)) {
        return false;
    }

    mapName = obj["Name"].asString();
    auto & JSnodeInses = obj["nodeInstance"];

    if (!checkJSMember({"No", "Exits"}, JSnodeInses[0])) {
        return false;
    }

    if (!checkJSMember({"dir", "pos"}, JSnodeInses[0]["Exits"][0])) {
        return false;
    }

    std::vector<NodeInstance *> nodeInses(JSnodeInses.size(), nullptr);
    for (int i = 0; i < JSnodeInses.size(); i++) {
        auto ins = new NodeInstance();
        auto & JSIns = JSnodeInses[i];
        auto & JSexits = JSIns["Exits"];
        for (int j = 0; j < JSexits.size(); j++) {
            auto & JSexit = JSexits[i];
            ins->addExit(JSexit["pos"][0].asDouble(),
                         JSexit["pos"][1].asDouble(),
                         JSexit["dir"].asDouble());
        }
        ins->completeAdding();
        int No = JSIns["No"].asInt();
        while (No > nodeInses.size()) {
            nodeInses.push_back(nullptr);
        }
        nodeInses[No] = ins;
    }

    //TODO input the maps

    return true;
}
