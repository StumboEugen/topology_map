//
// Created by stumbo on 18-9-18.
//

#include "TopoFile.h"
#include "TopoType.h"
#include <unistd.h>
#include <pwd.h>
#include <vector>

using std::string;

TopoFile::TopoFile(std::string fileName, bool save) {
    uid_t uid;
    struct passwd* pwd;
    uid = getuid();
    pwd = getpwuid(uid);
    chdir(pwd->pw_dir);

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

    if (slashes.empty()) {
        path = TOPO_STD_FILE_SAVE_FLODER_NAME + fileName;
        slashes.push_back(string(TOPO_STD_FILE_SAVE_FLODER_NAME).size() - 1);
    } else {
        path = fileName;
    }

    string floder;
    size_t lastSlashPos = 0;
    for (const auto spos : slashes) {
        floder = path.substr(lastSlashPos, spos - lastSlashPos);
        chdir(floder.c_str());
        mkdir(floder.c_str(), 0b111111111);
        lastSlashPos = spos + 1;
    }

    std::ofstream of;
    auto fm = path.substr(slashes.back() + 1);
    of.open(fm, std::ios::out); //TODO second try relating to 3 floders + can't create file
}
