//
// Created by stumbo on 18-5-10.
//

#ifndef TOPOLOGY_MAP_NODEEXIT_H
#define TOPOLOGY_MAP_NODEEXIT_H

#include <memory>

using std::shared_ptr

class TopoNode

class NodeExit {
public:
    double & Dir() {
        return dir;
    }

    shared_ptr<NodeExit> & LeadTo() {
        return leadTo;
    }

    shared_ptr<TopoNode> & ExitForm() {
        return exitFrom;
    }
private:
    double dir;
    shared_ptr<NodeExit> leadTo;
    shared_ptr<TopoNode> exitFrom;
};


#endif //TOPOLOGY_MAP_NODEEXIT_H
