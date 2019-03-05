//
// Created by stumbo on 18-9-18.
//

#ifndef TOPOLOGY_MAP_TOPOFILE_H
#define TOPOLOGY_MAP_TOPOFILE_H

#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>
#include <pwd.h>
#include <vector>
#include <string>

#include "MapArranger.h"
#include "MapCollection.h"
#include "MapCandidate.h"
#include "TopoEdge.h"
#include "TopoNode.h"
#include "NodeCollection.h"
#include "NodeInstance.h"

/**
 * a tool class for TopoMap IO
 */
class TopoFile {
public:
    /**
     * constructor, the fileName can be assigned
     * @param fileName the path of the file, default("") is ~/topoMaps/<current Time>.
     * absolute path is not supported
     * @example
     * filename = abc/map will create the map at ~/abc/map
     */
    explicit TopoFile(const std::string &fileName = "");
    /**
     * @param fileName the path of the file, default("") is ~/topoMaps/<current Time>.
     * absolute path is not supported
     * @example
     * filename = abc/map will create the map at ~/abc/map
     */
    int setFileName(std::string fileName);
    /**
     * open file at first, if fail, will automanticly turn to default path and name
     * @param mode out, trunc(delete at first), in
     * @return 0 if successful, -1 if failed(but the default setting will work)
     */
    int open(std::_Ios_Openmode mode = std::ios::out | std::ios::trunc);
    /**
     * write the mapGroup you assign to the file in JSON style
     * @param mapGroup the maps to record
     * @return 0 if success, -1 if fail
     */
    int outputMap(const MapArranger &mapGroup);

    int inputMap(MapArranger &mapGroup);

    int close();

    virtual ~TopoFile();

private:
//    std::string spliter = ",";
    std::string filePath;
    std::fstream fs;

    /**move to the user floder*/
    void chDir2UserPath();
};


#endif //TOPOLOGY_MAP_TOPOFILE_H
