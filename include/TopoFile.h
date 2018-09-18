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

class TopoFile {
public:
    explicit TopoFile(const std::string &fileName = "");
    int setFileName(std::string fileName);
    int open(std::_Ios_Openmode mode = std::ios::out | std::ios::trunc);

private:
    std::string filePath;
    std::fstream fs;

    /**move to the user floder*/
    void chDir2UserPath();
};


#endif //TOPOLOGY_MAP_TOPOFILE_H
