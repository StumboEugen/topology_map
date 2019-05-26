//
// Created by stumbo on 18-5-21.
//

#ifndef TOPOLOGY_MAP_MAPARRANGER_H
#define TOPOLOGY_MAP_MAPARRANGER_H

#include "TopoTools.h"
#include "MapCollection.h"
#include "NodeCollection.h"

/**
 * A boss class of the map part
 */
class MapArranger {
public:
    MapArranger();

    void arriveInstance(NodeInstance *instance, gateId arriveAt, double odomX, double odomY,
                            double yaw);
    void moveThroughGate(gateId gate);
    size_t getMapNumbers();
    size_t experienceNum() {
        return experiences;
    }

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

    /**
     * sort the map, if the count asked is 0, sort every map
     * @param topCount
     */
    void sortByConfidence(size_t topCount = 0);

    bool readFromJSON(const JSobj & obj);

    bool readFromStr(const std::string & str);


    /**
     * turn the map into a JSON obj
     * @param mapCount the number of mapCandidate you would like to save (0 == whole)
     * @return the js obj
     */
    JSobj toJS(size_t mapCount = 0);

    /**
     * turn the whold map into a str in JSON format
     * @param mapCount the number of mapCandidate you would like to save (0 == whole)
     * @return the str
     */
    string toString(size_t mapCount = 0);

    bool reloadFromFile(const std::string & fileName);

    void selfClean();

private:
    size_t experiences = 0;
    NodeCollection nodeCollection;
    MapCollection mapCollection;

    /**
     * the name of this map group, it should be "const" like
     */
    std::string mapName;
};


#endif //TOPOLOGY_MAP_MAPARRANGER_H
