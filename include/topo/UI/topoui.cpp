#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QPointF>
#include <QHash>
#include <QDebug>
#include <QSpacerItem>
#include <QSizePolicy>
#include <QGraphicsItem>
#include <QWheelEvent>

#include "topoui.h"
#include "ui_topoui.h"
#include "ui_dockreadmap.h"
#include "ui_dockbuildmap.h"
#include "topo/Topo.h"

#include "TopoMapGView.h"
#include "QGI_Node.h"
#include "QGI_Edge.h"

#include <map>
#include <queue>

using namespace std;

TopoUI::TopoUI(QWidget *parent) :
    QMainWindow(parent),
    uiMain(new Ui::TopoUI),
    uiDockReadMap(new Ui::DockReadMapUI),
    uiDockBuildMap(new Ui::DockBuildMapUI)
{
    uiMain->setupUi(this);

    modeGroup = new QActionGroup(this);
    mode_READ = modeGroup->addAction("read file mode");
    mode_BUILD = modeGroup->addAction("build map mode");
    mode_SIMULATION = modeGroup->addAction("simulation mode");
    mode_READ->setCheckable(true);
    mode_BUILD->setCheckable(true);
    mode_SIMULATION->setCheckable(true);
    mode_READ->setChecked(true);
    uiMain->mainToolBar->addAction(mode_READ);
    uiMain->mainToolBar->addAction(mode_BUILD);
    uiMain->mainToolBar->addAction(mode_SIMULATION);
    uiMain->mainToolBar->addSeparator();

    QAction * dragMode = new QAction("Drag Mode", uiMain->mainToolBar);
    dragMode->setCheckable(true);
    uiMain->mainToolBar->addAction(dragMode);
    uiMain->mainToolBar->addSeparator();
    connect(dragMode, SIGNAL(toggled(bool)), this, SLOT(setMapGViewDragMode(bool)));

    centerLayout = new QHBoxLayout(uiMain->centralWidget);
    centerLayout->setSpacing(6);
    centerLayout->setContentsMargins(11, 11, 11, 11);
    centerLayout->setObjectName(QString::fromUtf8("centerLayout"));

    mapGView = new TopoMapGView(uiMain->centralWidget);
    mapGView->setObjectName(QString::fromUtf8("mapGView"));
//    mapGView->setGeometry(QRect(10, 10, 601, 401));
    mapGView->setScene(&this->mapScene);
//    mapGView->setMaximumSize(601, 401);
    QSizePolicy p;
    p.setHorizontalPolicy(QSizePolicy::Expanding);
    p.setVerticalPolicy(QSizePolicy::Expanding);
    mapGView->setSizePolicy(p);
    mapGView->setMinimumSize(601, 401);

    centerLayout->addWidget(mapGView);

    smallWindowLayout = new QVBoxLayout();
    smallWindowLayout->setSpacing(6);
    smallWindowLayout->setContentsMargins(11, 11, 11, 0);
    smallWindowLayout->setObjectName(QString::fromUtf8("smallWindowLayout"));

    nodeGView = new TopoNodeGView(uiMain->centralWidget);
    nodeGView->setObjectName(QString::fromUtf8("nodeGView"));
//    nodeGView->setGeometry(QRect(620, 191, 221, 221));
    nodeGView->setScene(&this->nodeScene);
    nodeGView->setFixedSize(221, 221);
//    nodeGView->setMaximumSize(221, 221);
//    nodeGView->setMinimumSize(221, 221);
    nodeGView->setCursor(Qt::CrossCursor);

    smallWindowLayout->addWidget(nodeGView, 0, Qt::AlignBottom);

    centerLayout->addLayout(smallWindowLayout);

    setDockNestingEnabled(true);
    dockReadMap = initTheDock("DockReadMap");
    uiDockReadMap->setupUi(dockReadMap);
    addDockWidget(Qt::RightDockWidgetArea, dockReadMap);

    dockBuildMap = initTheDock("DockBuildMap");
    uiDockBuildMap->setupUi(dockBuildMap);
    addDockWidget(Qt::RightDockWidgetArea, dockBuildMap);
    dockBuildMap->setShown(false);

//    uiDockBuildMap->setupUi(dockReadMap);//TODO

    connect(uiDockReadMap->btnInputMap, SIGNAL(clicked())
            , this, SLOT(loadMapFromFile()));

    connect(uiDockReadMap->cmboMapCandidate, SIGNAL(activated(int))
            , this, SLOT(displayTheActivitedMap(int)));

    connect(mapGView, SIGNAL(QGI_Node_clicked(TopoNode*))
            , this, SLOT(drawTopoNode(TopoNode*)));

    connect(modeGroup, SIGNAL(triggered(QAction*))
            , this, SLOT(changeMode(QAction*)));
}

TopoUI::~TopoUI()
{
    delete uiMain;
}

void TopoUI::loadMapFromFile() {
    mapScene.clear();

//    mapScene.removeItem(mapScene.itemAt({50, 50}));

    if (mapGroup.reloadFromFile(uiDockReadMap->etInputMap->text().toStdString())) {
        cout << "UI load map successful" << endl;
        cleanEveryThing();
        int mapCounts = 0;

        for (const auto & mapCand: mapGroup.getMapCollection().getMaps()) {
            QString comboInfo = QString("#%1 fullEdge:%2")
                    .arg(mapCounts).arg(mapCand->getFullEdgeNumber());
            comboBoxMaps.push_back(mapCand);
            uiDockReadMap->cmboMapCandidate->addItem(comboInfo);
            mapCounts++;
        }

    } else {
        cerr << "UI load map FAIL" << endl;
    }
}

void TopoUI::displayTheActivitedMap(int index) {
    mapScene.clear();

    MapCandidate & map2Draw = *comboBoxMaps[index];
    map2Draw.cleanAllNodeFlags();

    auto beginNode = map2Draw.getOneTopoNode();

    queue<TopoNode*> lookupQueue;

    auto nodeQGI = new QGI_Node(beginNode);
    beginNode->setAssistPtr(nodeQGI);
    mapScene.addItem(nodeQGI);
    lookupQueue.push(beginNode);

    while(!lookupQueue.empty()) {
        auto & curNode = lookupQueue.front();
        lookupQueue.pop();
        QPointF curPos = static_cast<QGI_Node*>(curNode->getAssistPtr())->pos();

        //look into every edge
        for (gateId edgeNo = 0; edgeNo < curNode->getInsCorrespond()->sizeOfExits(); edgeNo++) {

//            //means this edge have been drawn
//            if (curNode->chkFlag(edgeNo)) {
//                continue;
//            }

            auto curEdge = curNode->getEdge(edgeNo);
            //means this edge has never been moved through
            if (curEdge == nullptr) {
                continue;
            }

            TopoNode * anotherNode = curEdge->getAnotherNode(curNode);
            anotherNode->setFlag(curEdge->getAnotherGate(curNode));

            //means the "anotherNode" has been drawn
            if (anotherNode->getAssistPtr() != nullptr) {
                auto edge2Draw = new QGI_Edge(curEdge
                        , static_cast<QGI_Node *>(curNode->getAssistPtr())
                        , static_cast<QGI_Node *>(anotherNode->getAssistPtr()));
                mapScene.addItem(edge2Draw);
                continue;
            }

            //draw another node
            auto odomData = curEdge->getOdomData(curNode);
            QPointF dist{odomData.first, -odomData.second}; //ENU is different with the UI coor
            nodeQGI = new QGI_Node(anotherNode);
            anotherNode->setAssistPtr(nodeQGI);
            nodeQGI->setPos(curPos + dist * METER_TO_PIXLE);
            mapScene.addItem(nodeQGI);

            lookupQueue.push(anotherNode);
        }
    }

    for (const auto & node: map2Draw.getNodes()) {
        auto tempIns = node->getInsCorrespond();
        QString tempStr = "(";
        for (const auto exit: tempIns->getExits()) {
            tempStr.append(QString::number(exit.getOutDir()) + ",");
        }
        tempStr.append(")");
        qDebug() << tempStr << ", " << static_cast<QGI_Node*>(node->getAssistPtr())->pos();
    }

//    cout << "it complete" << endl;
}

void TopoUI::drawTopoNode(TopoNode * topoNode) {
    nodeScene.clear();
    auto QNode = new QGI_Node(topoNode);
    QNode->setDrawDetail(true);
    nodeScene.addItem(QNode);
}

void TopoUI::cleanEveryThing() {
    mapScene.clear();
    nodeScene.clear();
    uiDockReadMap->cmboMapCandidate->clear();
    comboBoxMaps.clear();
}

void TopoUI::changeMode(QAction * action) {
    if (action == mode_READ) {
        cout << "switch to read mode" << endl;
        dockReadMap->setShown(true);
    } else {
        dockReadMap->setShown(false);
    }

    if (action == mode_BUILD) {
        cout << "switch to build mode" << endl;
        dockBuildMap->setShown(true);
        cleanEveryThing();
    } else {
        dockBuildMap->setShown(false);
    }

    if (action == mode_SIMULATION) {
        cout << "switch to simulation mode" << endl;
    }
}

void TopoUI::setMapGViewDragMode(bool isDrag) {
    if (isDrag) {
        mapGView->setDragMode(QGraphicsView::ScrollHandDrag);
    } else {
        mapGView->setDragMode(QGraphicsView::NoDrag);
    }
}

QDockWidget *TopoUI::initTheDock(const char *name) {
    auto theDock = new QDockWidget(this);
    theDock->setObjectName(QString::fromUtf8(name));  //TODO
    QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Maximum);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(theDock->sizePolicy().hasHeightForWidth());
    theDock->setSizePolicy(sizePolicy1);
    theDock->setFloating(false);
    theDock->setFeatures(QDockWidget::DockWidgetFloatable|QDockWidget::DockWidgetMovable);
    theDock->setAllowedAreas(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea);
    return theDock;
}
