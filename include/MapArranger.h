//
// Created by stumbo on 18-5-21.
//

#ifndef TOPOLOGY_MAP_MAPARRANGER_H
#define TOPOLOGY_MAP_MAPARRANGER_H

#include "TopoType.h"
#include "MapCollection.h"
#include "NodeCollection.h"

class NodeInstance;

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
