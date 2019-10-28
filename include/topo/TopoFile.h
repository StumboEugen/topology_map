#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
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
 * @brief a tool class for TopoMap IO.
 * @note
 * \b step1 set the file name, the default file would be at ~/topoMaps/<current Time> <br>
 * \b step2 call open <br>
 * \b step3 call outputMap or intputMap
 *
 * @see open()
 * @see outputMap()
 * @see inputMap()
 */
class TopoFile {
public:
    /// the output mode, would remove the origin file
    const static std::_Ios_Openmode outputMapMode = std::ios::out | std::ios::trunc;
    /// the input mode
    const static std::_Ios_Openmode inputMapMode = std::ios::in;

public:
    // constructor, the fileName can be assigned
    explicit TopoFile(const std::string &fileName = "");

    // set the file name
    int setFileName(std::string fileName);

    // open file
    int open(std::_Ios_Openmode mode = outputMapMode);

    //write the mapGroup you assign to the file in JSON style
    int outputMap(MapArranger & mapGroup);

    // input the related file to assigned MapArranger
    int inputMap(MapArranger &mapGroup);

    int close();

    virtual ~TopoFile();

    void setSpliter(const string & spliter) {
        TopoFile::spliter = spliter;
    }

private: // functions
    // move the current dir to user's floder ~/
    void chDir2UserPath();

private: // members
    /// the spliter used in JSON file, for compress, use ""
    std::string spliter = "\t";

    /// the file's path
    std::string filePath;

    /// the file stream to arrange the output and input
    std::fstream fs;
};


#endif //TOPOLOGY_MAP_TOPOFILE_H

#pragma clang diagnostic pop