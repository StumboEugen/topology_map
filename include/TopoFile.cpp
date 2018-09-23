//
// Created by stumbo on 18-9-18.
//

#include "TopoFile.h"
#include "TopoTools.h"
#include <unistd.h>
#include <pwd.h>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

TopoFile::TopoFile(const string &fileName) {
    /**move to the user floder*/
    setFileName(fileName);
}

void TopoFile::chDir2UserPath() {
    uid_t uid;
    struct passwd* pwd;
    uid = getuid();
    pwd = getpwuid(uid);
    chdir(pwd->pw_dir);
}

int TopoFile::setFileName(string fileName) {

    chDir2UserPath();

    /**fix some names*/
    if (!fileName.empty()) {
        if (fileName[0] == '~') {
            fileName.erase(0,1);
        }
    }

    if (!fileName.empty()) {
        if (fileName[0] == '/') {
            fileName.erase(0,1);
        }
    }

    /**get the '/'s in the fileName*/
    std::vector<size_t > slashes;
    auto eos = fileName.size();
    size_t slashPos = 0;
    while (true) {
        slashPos = fileName.find('/', ++slashPos);
        if (slashPos < eos) {
            slashes.push_back(slashPos);
        } else {
            break;
        }
    }

    /**reslove the default path of the file*/
    if (slashes.empty()) {
        filePath = TOPO_STD_FILE_SAVE_FLODER_NAME + fileName;
        std::cout << "You didn't assign the floder, "
                     "use the default: ~/" << TOPO_STD_FILE_SAVE_FLODER_NAME << std::endl;
        if (fileName.empty()) {
            std::cout << "You didn't assign the fileName, "
                         "use the default: unamedMap" << topo::getCurrentTimeString()
                      << std::endl;
            filePath.append("unamedMap_" + topo::getCurrentTimeString());
        }
        slashes.push_back(string(TOPO_STD_FILE_SAVE_FLODER_NAME).size() - 1);
    } else {
        filePath = fileName;
    }

    /**build the floders resovly*/
    string floder;
    size_t lastSlashPos = 0;
    for (const auto spos : slashes) {
        floder = filePath.substr(lastSlashPos, spos - lastSlashPos);
        if (access(floder.c_str(), F_OK) != 0) {
            mkdir(floder.c_str(), 0b111111111);
        }
        if (chdir(floder.c_str()) == -1 || floder == "/") {
            std::cout << "\n[ERROR] FAIL TO OPEN FILE: ~/" + filePath
                      << "\n        Stuck at:" << floder
                      << "\n        fall back to the default path"
                      << std::endl;
            setFileName("");
            return -1;
        }
        lastSlashPos = spos + 1;
    }
    return 0;
}

int TopoFile::open(std::_Ios_Openmode mode) {
    chDir2UserPath();
    fs.open(filePath, mode);
    if (fs.is_open()) {
        cout << "open success!\n" << filePath << endl;
        return 1;
    } else {
        cerr << "open FAIL, fall back to default mode" << endl;
        setFileName("");
        open();
    }

    return -1;
}

int TopoFile::outputMap(const MapArranger &mapGroup) {
    if (!fs.is_open()) {
        cout << "[TopoFile::outputMap] you didn't open at first! "
                "Try to open now ..." << endl;
        open();
    }

    fs << mapGroup.getMapName() << endl;

    const auto & nodeSets = mapGroup.getNodeCollection().getNodeSets();
    for (const auto & nodeSet: nodeSets) {
        for (const auto & nodeIns : nodeSet.second) {
            fs << nodeIns->getSerialNumber() << spliter;
//            fs << nodeIns->
        }
    }
    return 0;
}
