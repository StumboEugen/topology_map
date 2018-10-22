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

    void arriveInstance(NodeInstance * instance, gateId arriveAt,
                        double odomX, double odomY);
    void moveThroughGate(gateId gate);
    size_t getMapNumbers();
    size_t experienceNum() {
        return experiences;
    }

    const string &getMapName() const {
        return mapName;
    }

    NodeCollection &getNodeCollection()  {  //TODO const?
        return nodeCollection;
    }

    const MapCollection &getMapCollection() const {
        return mapCollection;
    }

    bool readFromJSON(const JSobj & obj);

    JSobj toJS() const;

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
