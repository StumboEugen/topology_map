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

UIMode CURRENT_MODE = READ_MODE;

TopoUI::TopoUI(QWidget *parent) :
    QMainWindow(parent),
    uiMain(new Ui::TopoUI),
    uiDockReadMap(new Ui::DockReadMapUI),
    uiDockBuildMap(new Ui::DockBuildMapUI)
{
    uiMain->setupUi(this);
    setWindowTitle("TOPO Simulator");

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
    mapGView->setScene(&this->mapScene);
    QSizePolicy p;
    p.setHorizontalPolicy(QSizePolicy::Expanding);
    p.setVerticalPolicy(QSizePolicy::Expanding);
    mapGView->setSizePolicy(p);
    mapGView->setMinimumSize(601, 401);

    centerLayout->addWidget(mapGView);

    smallWindowLayout = new QVBoxLayout();
    smallWindowLayout->setSpacing(11);
    smallWindowLayout->setContentsMargins(11, 0, 11, 0);
    smallWindowLayout->setObjectName(QString::fromUtf8("smallWindowLayout"));

    infoView = new QTextBrowser(uiMain->centralWidget);
    infoView->setObjectName(QString::fromUtf8("infoView"));
    infoView->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    infoView->setBaseSize(221, 221);
    infoView->setText("Welcome!");
    smallWindowLayout->addWidget(infoView, 0, Qt::AlignTop);

    nodeGView = new TopoNodeGView(uiMain->centralWidget);
    nodeGView->setObjectName(QString::fromUtf8("nodeGView"));
    nodeGView->setScene(&this->nodeScene);
    nodeGView->setFixedSize(221, 221);
    nodeGView->setCursor(Qt::CrossCursor);
    nodeGView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);

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

    connect(uiDockReadMap->btnInputMap, SIGNAL(clicked())
            , this, SLOT(loadMapFromFile()));

    connect(uiDockReadMap->cmboMapCandidate, SIGNAL(activated(int))
            , this, SLOT(displayTheActivitedMap(int)));

    connect(mapGView, SIGNAL(QGI_Node_clicked(TopoNode*))
            , this, SLOT(drawTopoNodeDetailAtnodeGView(TopoNode * )));

    connect(modeGroup, SIGNAL(triggered(QAction*))
            , this, SLOT(changeMode(QAction*)));

    connect(uiDockBuildMap->btnMakeNewNode, SIGNAL(clicked())
            , this, SLOT(buildModeNewNode()));

    connect(uiDockBuildMap->btnAddNodeIntoMap, SIGNAL(clicked())
            , this, SLOT(buildModeAddNode2MapView()));

    connect(uiDockBuildMap->btnDrawEdge, SIGNAL(clicked(bool))
            , mapGView, SLOT(switch2DrawEdgeMode(bool)));

    connect(mapGView, SIGNAL(newEdgeConnected(TopoEdge *))
            , this, SLOT(newEdgeConnected(TopoEdge *)));

}

TopoUI::~TopoUI()
{
    delete uiMain;
}

void TopoUI::loadMapFromFile() {
    cleanEveryThing(); //TODO

    if (mapFromReading.reloadFromFile(uiDockReadMap->etInputMap->text().toStdString())) {
        cout << "UI load map successful" << endl;

        int mapCounts = 0;

        for (const auto & mapCand: mapFromReading.getMapCollection().getMaps()) {
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
            uint8_t anotherGate = curEdge->getAnotherGate(curNode);
            anotherNode->setFlag(anotherGate);

            //means the "anotherNode" has been drawn
            if (anotherNode->getAssistPtr() != nullptr) {
                auto edge2Draw = new QGI_Edge(curEdge,
                        static_cast<QGI_Node *>(curNode->getAssistPtr()),
                        static_cast<QGI_Node *>(anotherNode->getAssistPtr()));
                mapScene.addItem(edge2Draw);
                continue;
            }

            //draw another node
            auto odomData = curEdge->getOdomData(curNode);
            QPointF dist{odomData.first, -odomData.second}; //ENU is different with the UI coor
//            const auto exitOfAnotherSide =
//                    anotherNode->getInsCorrespond()->getExits()[anotherGate];
//            QPointF disInAnotherNode{exitOfAnotherSide.getPosX(), -exitOfAnotherSide.getPosY()};
//            dist += disInAnotherNode;
            //TODO up
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

void TopoUI::drawTopoNodeDetailAtnodeGView(TopoNode *topoNode) {
    if (CURRENT_MODE == READ_MODE) {
        nodeScene.clear();
        auto QNode = new QGI_Node(topoNode);
        QNode->setDrawDetail(true);
        nodeScene.addItem(QNode);
    }
}

void TopoUI::cleanEveryThing() {
    mapScene.clear();
    nodeScene.clear();
    mapFromReading.selfClean();
    uiDockReadMap->cmboMapCandidate->clear();   //TODO use QVariant
    comboBoxMaps.clear();   //TODO WARNNING leak
    buildModeNodes.clear();
}

void TopoUI::changeMode(QAction * action) {
    if (action == mode_READ) {
        cout << "switch to read mode" << endl;
        CURRENT_MODE = READ_MODE;
        dockReadMap->setShown(true);
        cleanEveryThing();
    } else {
        dockReadMap->setShown(false);
    }

    if (action == mode_BUILD) {
        cout << "switch to build mode" << endl;
        CURRENT_MODE = BUILD_MODE;
        dockBuildMap->setShown(true);
        uiDockBuildMap->btnMakeNewNode->setText("make a new node");
        cleanEveryThing();
    } else {
        dockBuildMap->setShown(false);
    }

    if (action == mode_SIMULATION) {
        cout << "switch to simulation mode" << endl;
        CURRENT_MODE = SIMULATION_MODE;
    }
}

void TopoUI::setMapGViewDragMode(bool isDrag) {
    if (isDrag) {
        mapGView->setDragMode(QGraphicsView::ScrollHandDrag);
    } else {
        mapGView->setDragMode(QGraphicsView::NoDrag);
    }
}

QDockWidget *TopoUI::initTheDock(const char *objectName) {
    auto theDock = new QDockWidget(this);
    theDock->setObjectName(QString::fromUtf8(objectName));
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

void TopoUI::buildModeNewNode() {
    auto pIns = nodeGView->getTheDrawnInstance();
    /**it is not at building ins, start building*/
    if (pIns == nullptr) {
        nodeGView->startDrawingIns();
        uiDockBuildMap->btnMakeNewNode->setText("finish drawing");
        uiDockBuildMap->cmboMadeNodes->setEnabled(false);
        uiDockBuildMap->btnAddNodeIntoMap->setEnabled(false);
    /**it was at building, stop building, save the node*/
    } else {
        if (pIns->sizeOfExits() != 0) {
            QVariant thePtrVariant = QVariant::fromValue((void*)pIns);
            auto nameStr = QString("#%1 E%2[")
                    .arg(uiDockBuildMap->cmboMadeNodes->count() + 1)
                    .arg(pIns->sizeOfExits());
            for (const auto & exit : pIns->getExits()) {
                double ang = atan2(exit.getPosY(), exit.getPosX()) * RAD2DEG;
                nameStr += QString::number(static_cast<int>(ang)) + "|";
            }
            nameStr += "]";
            mapFromBuilding.addInstanceDirectly(pIns);
            uiDockBuildMap->cmboMadeNodes->addItem(nameStr, thePtrVariant);
        }
        uiDockBuildMap->btnMakeNewNode->setText("make a new node");
        uiDockBuildMap->cmboMadeNodes->setEnabled(true);
        uiDockBuildMap->btnAddNodeIntoMap->setEnabled(true);
    }

}

void TopoUI::buildModeAddNode2MapView() {
    auto pBox = uiDockBuildMap->cmboMadeNodes;
    NodeInstance * pIns =
            static_cast<NodeInstance *>(pBox->itemData(pBox->currentIndex()).value<void*>());
    auto pnode = new TopoNode(pIns);
    mapFromBuilding.addTopoNodeDirectly(pnode);
    auto QGI_node = new QGI_Node(pnode);
    QGI_node->setDrawDetail(true);
    QGI_node->setFlag(QGraphicsItem::ItemIsMovable);
    QGI_node->setAcceptDrops(true);
    mapGView->scene()->addItem(QGI_node);
}

void TopoUI::newEdgeConnected(TopoEdge * newEdge) {
    mapFromBuilding.getMapCollection().getMaps().front()->addEdgeDirectly(newEdge);
}

