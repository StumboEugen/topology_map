//
// Created by stumbo on 18-9-18.
//

#include <string>
#include <ctime>
#include "TopoTools.h"

using namespace std;

const std::string topo::getCurrentTimeString() {
    struct tm * timeStructP;
    time_t timeLong;
    timeLong = time(nullptr);
    timeStructP = localtime(&timeLong);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M", timeStructP);
    return move(string(tmp));
}

bool topo::checkJSMember(const vector<std::string> &strs, const JSobj &js) {
    for (const auto & str: strs) {
        if(js.isMember(str)) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}
