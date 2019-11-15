//
// Created by stumbo on 18-5-21.
//

#ifndef TOPOLOGY_MAP_MAPARRANGER_H
#define TOPOLOGY_MAP_MAPARRANGER_H

#include "TopoTools.h"
#include "MapCollection.h"
#include "NodeCollection.h"

/**
 * @brief The surface class of the Topo map core
 * contain a NodeCollection and a MapCollection.
 */
class MapArranger {
public:
    // constructor of MapArranger, a name is required to assign name
    explicit MapArranger(const std::string & mapName = topo::getCurrentTimeString());

    // tell the core that we arrive at a new NodeInstance
    void arriveInstance(NodeInstance *instance, gateId arriveAt, double odomX, double odomY,
                            double yaw);

    // tell the core that the robot moves through a gate
    void moveThroughGate(gateId gate);

    // get the size of maps
    size_t getMapNumbers();

    // sort the maps according the confidence.
    void sortByConfidence(size_t topCount = 0);

    // input according to a JSON structure
    bool readFromJSON(const JSobj & obj);

    // input according to a JSON structure in string format
    bool readFromStr(const std::string & str);

    // convert the built possibile maps into JSON structure
    JSobj toJS(size_t mapCount = 0);

    // turn the whold map into a str in JSON format
    string toString(size_t mapCount = 0);

    // load the file according to the file name
    bool reloadFromFile(const std::string & fileName);

    // clean everything, data, ptrs
    void selfClean();

    // 当前已经走过的 NodeInstance 数量
    size_t experienceNum();

    const string &getMapName() const {
        return mapName;
    }

    void setMapName(const string & mapName) {
        MapArranger::mapName = mapName;
    }

    const NodeCollection &getNodeCollection() const {
        return nodeCollection;
    }

    MapCollection & getMapCollection() {
        return mapCollection;
    }

    void addInstanceDirectly(NodeInstance * ins) {
        nodeCollection.addInstanceDirectly(ins);
    }

    /// this is used for UI building mode
    void addTopoNodeDirectly(TopoNode * node) {
        mapCollection.addNodeDirectly(node);
    }

    void addTopoEdgeDirectly(TopoEdge * edge) {
        mapCollection.addEdgeDirectly(edge);
    }

private:

    /// the NodeCollection
    NodeCollection nodeCollection;

    /// the MapCollection
    MapCollection mapCollection;

    /// name of the whole map
    std::string mapName;
};


#endif //TOPOLOGY_MAP_MAPARRANGER_H
