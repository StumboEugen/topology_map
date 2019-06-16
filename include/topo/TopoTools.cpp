//
// Created by stumbo on 18-9-18.
//

#include <string>
#include <ctime>
#include "TopoTools.h"
#include <cmath>

using namespace std;

/**
 * @brief a tool to get a string represents time (only in LINUX)
 * @return a string represents time like yyyymmdd_time \b example: 20190614_1523
 */
const std::string topo::getCurrentTimeString() {
    struct tm * timeStructP;
    time_t timeLong;
    timeLong = time(nullptr);
    timeStructP = localtime(&timeLong);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M", timeStructP);
    return move(string(tmp));
}

/**
 * @brief check if the required names is a member of the JSON object
 * @param strs a list of string to check
 * @param js the target JSON object
 * @return if the JSON object has all of these string members
 */
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

/**
 * @brief calculate the sqrt (x^2 + y^2)
 */
double topo::calDis(double x, double y) {
    return sqrt(x * x + y * y);
}

/**
 * @brief convert a number to [-pi, pi] in a cycle of 2pi
 * @note we didn't consider the number < -3pi and > 3pi
 */
double topo::fixRad2nppi(double target) noexcept {
    if (target > pi) {
        target -= piTwo;
    }
    else if (target < -pi) {
        target += piTwo;
    }
    return target;
}
