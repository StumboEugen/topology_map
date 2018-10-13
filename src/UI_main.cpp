//
// Created by stumbo on 18-9-27.
//

#include "topo/UI/topoui.h"
#include <QApplication>

int main(int argc, char **argv) {

    /**
    * without these lines, debug will fail
    */
    auto xxxxx = new NodeInstance();
    std::list<MapCandidate*> mm;
    auto forDebug = new MapCandidate(xxxxx);
    mm.push_back(forDebug);

    QApplication app(argc, argv);
    TopoUI theUi;
    theUi.show();
    app.exec();
}