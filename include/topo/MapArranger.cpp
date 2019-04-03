//
// Created by stumbo on 18-5-21.
//
#include <iostream>

using namespace std;

#include "Topo.h"
#include "MapArranger.h"

void MapArranger::arriveInstance(NodeInstance *instance, gateId arriveAt,
                                 double odomX, double odomY, double yaw) {
    if (!instance->isAddComplete()) {
        cout << "[MapArranger::arriveInstance] "
                "You add a uncompleted node to the collection!" << endl;
    }
    /**
     * firstly apply this move to every map
     */
    mapCollection.arriveNodeInstance(instance, arriveAt, odomX, odomY, yaw);
    /**
     * secondly find the similiar ones
     */
    auto newMaps = nodeCollection.addInstanceAndCompare(instance, arriveAt, odomX, odomY);

    for (const auto & newMap: newMaps) {
        mapCollection.addNewMap(newMap);
//        newMap.second->setPosInList(newPos);
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
        : mapName(topo::getCurrentTimeString()),
          mapCollection(this),
          nodeCollection(this)
{}

JSobj MapArranger::toJS(size_t mapCount) {
    JSobj obj;
    obj["nodeInstance"] = nodeCollection.toJS();
    obj["mapInstance"] = mapCollection.toJSWithSortedMaps(mapCount);
    obj["Name"] = mapName;
    return std::move(obj);
}

using topo::checkJSMember;

/**
 * decode from a JSON obj
 * @param obj the JSON obj
 * @return true if success, false if unsuccess
 */
bool MapArranger::readFromJSON(const JSobj &obj) {
    selfClean();

    /**
     * verify the JS structor
     */
    if (!checkJSMember({"Name", "nodeInstance", "mapInstance"},obj)) {
        return false;
    }

    auto & JSnodeInses = obj["nodeInstance"];

    if (!checkJSMember({"No", "Exits"}, JSnodeInses[0])) {
        return false;
    }

    if (!checkJSMember({"dir", "pos"}, JSnodeInses[0]["Exits"][0])) {
        return false;
    }

    auto & JSmaps = obj["mapInstance"];
    if (!checkJSMember({"curNode", "edgeFullNum", "edges", "lEiOe"}, JSmaps[0])) {
        return false;
    }

    if (!JSmaps[0]["edges"].empty()) {
        if (!checkJSMember({"Ea", "Eb", "Ga", "Gb", "Oa", "Ox", "Oy"},
                           JSmaps[0]["edges"][0])) {
            return false;
        }

    }
    /**
     * input the name
     */
    mapName = obj["Name"].asString();

    /**
     * build the serialNumber-Instance pair dict
     * TODO not robust
     */
    try {
//        std::vector<NodeInstance *> nodeInsesDict(JSnodeInses.size(), nullptr);
        std::vector<NodeInstance *> nodeInsesDict;
        for (int i = 0; i < JSnodeInses.size(); i++) {
            auto ins = new NodeInstance(false);
            auto & JSIns = JSnodeInses[i];
            auto & JSexits = JSIns["Exits"];
            for (int j = 0; j < JSexits.size(); j++) {
                auto & JSexit = JSexits[j];
                ins->addExit(JSexit["pos"][0].asDouble(),
                             JSexit["pos"][1].asDouble(),
                             JSexit["dir"].asDouble());
            }
            ins->completeAdding();
            int No = JSIns["No"].asInt();
            while (No + 1 > nodeInsesDict.size()) {
                nodeInsesDict.push_back(nullptr);
            }
            nodeInsesDict[No] = ins;
            ins->setSerialNumber(static_cast<size_t>(No));
        }

        /**
         * construct the map collection using the serialNO-INS dict
         */
        for (int i = 0; i < JSmaps.size(); i++) {
            auto newMap = new MapCandidate(nodeInsesDict, JSmaps[i]);
            mapCollection.addNewMap(newMap);
//            newMap->setPosInList(pos);
        }

        /**
         * construct the node collection
         */
        for (const auto & theIns: nodeInsesDict) {
            if (theIns != nullptr) {
                nodeCollection.addInstanceDirectly(theIns);
            }
        }
    } catch (exception & e) {
        cerr << "decode JSON file FAIL!\n" << e.what() << endl;
        return false;
    }
    return true;
}

/**
 * load the file according to he file name ~/topoMaps/<fileName>
 * if fail, the whole group will be cleared
 * @param fileName
 * @return if the reload is successful
 */
bool MapArranger::reloadFromFile(const std::string & fileName) {
    TopoFile existFile(fileName);
    if (existFile.open(std::ios::in) != 0) {
        cerr << "[MapArranger::reloadFromFile] "
                "open file failure, fall back to empty map" << endl;
        selfClean();
        return false;
    }
    if(existFile.inputMap(*this) != 0) {
        selfClean();
        return false;
    } else {
        return true;
    }
}

/**
 * clean everything, data, ptrs
 */
void MapArranger::selfClean() {
    mapCollection.clear();
    nodeCollection.clear();
}

void MapArranger::sortByConfidence(size_t topCount) {
        mapCollection.sortByConfidence(topCount);
}

string MapArranger::toString(size_t mapCount) {
    stringstream ss;
    Json::StreamWriterBuilder builder;
    builder["precision"] = 4;
    builder["indentation"] = "";
    std::unique_ptr<Json::StreamWriter> const writer(builder.newStreamWriter());
    writer->write(toJS(mapCount), &ss);
    return std::move(ss.str());
}

bool MapArranger::readFromStr(const std::string & str) {
    selfClean();
    Json::CharReaderBuilder b;
    char const* begin = str.data();
    char const* end = begin + str.size();
    std::unique_ptr<Json::CharReader> const reader(b.newCharReader());
    JSobj js;
    string err;
    bool ok = reader->parse(begin, end, &js, &err);
    if (ok) {
        return readFromJSON(js);
    } else {
        return false;
    }
}
