//
// Created by stumbo on 18-9-18.
//

#ifndef TOPOLOGY_MAP_TOPOFILE_H
#define TOPOLOGY_MAP_TOPOFILE_H

#include <iostream>
#include <fstream>
#include <sys/stat.h>

class TopoFile {
public:
    TopoFile(std::string fileName, bool save = true);

private:
    std::string path;
};


#endif //TOPOLOGY_MAP_TOPOFILE_H
