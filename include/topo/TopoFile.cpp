//
// Created by stumbo on 18-9-18.
//

#include "TopoFile.h"
#include "TopoTools.h"
#include <unistd.h>
#include <pwd.h>
#include <vector>
#include <iostream>

using namespace std;

/**
 * @brief constructor, the file's Name can be assigned
 * @param fileName the path of the file, default("") is ~/topoMaps/<current Time>.
 * absolute path is not supported
 * @note \b example:
 * filename = abc/map will create the map at ~/abc/map
 */
TopoFile::TopoFile(const string &fileName) {
    /**move to the user floder*/
    setFileName(fileName);
}

/// move the current dir to user's floder ~/
void TopoFile::chDir2UserPath() {
    uid_t uid;
    struct passwd* pwd;
    uid = getuid();
    pwd = getpwuid(uid);
    chdir(pwd->pw_dir);
}

/**
 * @brief set the file path
 * @param fileName the path of the file, default("") is ~/topoMaps/<current Time>.
 * absolute path is not supported
 * @note \b example:
 * filename = abc/map will create the map at ~/abc/map
 */
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
            std::cout << "\n[ERROR] FAIL TO REACH FLODER: ~/" + filePath
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

/**
 * @brief open file
 * you should open file at first, if fail, will automanticly turn to default path and name
 * @param mode outputMapMode, inputMapMode
 * @return 0 if successful, -1 if failed(but would fall back to default setting)
 */
int TopoFile::open(std::_Ios_Openmode mode) {
    chDir2UserPath();
    fs.open(filePath, mode);
    if (fs.is_open()) {
        cout << "open success!\n" << filePath << endl;
        return 0;
    } else {
        cerr << "open FAIL" << endl;
//        setFileName("");
//        open(mode);
        return -1;
    }
}

/**
 * write the mapGroup you assign to the file in JSON style
 * @param mapGroup the maps to record
 * @return 0 if success, -1 if fail
 * @see MapArranger::toJS()
 * @attention you can set the spliter
 * @see spliter
 */
int TopoFile::outputMap(MapArranger & mapGroup) {
    if (!fs.is_open()) {
        cout << "[TopoFile::outputMap] you didn't open at first! "
                "Try to open now ..." << endl;
        if ( open(outputMapMode) != 0) {
            return -1;
        }
    }
    Json::StreamWriterBuilder builder;
    builder["precision"] = 4;
    builder["indentation"] = spliter;
    auto writer(builder.newStreamWriter());
    writer->write(mapGroup.toJS(), &fs);
    fs << endl;
    return 0;
}

/**
 * @brief input the related file to assigned MapArranger
 * @param mapGroup the assigned MapArranger to input
 * @return if read successful, return 0 <br>
 * if file didn't open, return -1 <br>
 * if read failure, return -2
 * @see setFileName()
 */
int TopoFile::inputMap(MapArranger &mapGroup) {
    if (!fs.is_open()) {
        cout << "[TopoFile::inputMap] you didn't open at first! "
                "Try to open now ..." << endl;
        if (open(inputMapMode) != 0) {
            return -1;
        }
    }
    JSobj jsobj;
    fs >> jsobj;
    if(mapGroup.readFromJSON(jsobj)) {
        return 0;
    } else {
        return -2;
    }

}

int TopoFile::close() {
    fs.close();
}

TopoFile::~TopoFile() {
    this->close();
}
