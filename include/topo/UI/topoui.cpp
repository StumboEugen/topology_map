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
#include <QValidator>

#include "topoui.h"
#include "ui_topoui.h"
#include "ui_dockreadmap.h"
#include "ui_dockbuildmap.h"
#include "ui_docksimulation.h"
#include "topo/Topo.h"

#include "TopoMapGView.h"
#include "QNode.h"
#include "QEdge.h"
#include "QRobot.h"

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <map>
#include <queue>
#include <std_msgs/UInt8.h>
#include <random>
#include <ctime>

using namespace std;

UIMode CURRENT_MODE = READ_MODE;
TopoUI * bigBrother;

TopoUI::TopoUI(QWidget *parent) :
    QMainWindow(parent),
    uiMain(new Ui::TopoUI),
    uiDockReadMap(new Ui::DockReadMapUI),
    uiDockBuildMap(new Ui::DockBuildMapUI),
    uiDockSimulation(new Ui::DockSimulationUI)
{
    bigBrother = this;

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
    QRegExp regx("[0-9]+$");
    uiDockReadMap->leMapID->setValidator(new QRegExpValidator(regx, this));
    addDockWidget(Qt::RightDockWidgetArea, dockReadMap);

    dockBuildMap = initTheDock("DockBuildMap");
    uiDockBuildMap->setupUi(dockBuildMap);
    regx.setPattern("[0-9\\.-,]+$");
    uiDockBuildMap->leEdgeOdom->setValidator(new QRegExpValidator(regx, this));
    regx.setPattern("[0-9]+$");
    uiDockBuildMap->leRotation->setValidator(new QRegExpValidator(regx, this));
    addDockWidget(Qt::RightDockWidgetArea, dockBuildMap);
    dockBuildMap->setShown(false);

    dockSimulation = initTheDock("DockSimulation");
    uiDockSimulation->setupUi(dockSimulation);
    regx.setPattern("[0-9\\.]+$");
    uiDockSimulation->leEdgeNoise->setValidator(new QRegExpValidator(regx, this));
    uiDockSimulation->leNodeNoise->setValidator(new QRegExpValidator(regx, this));
    addDockWidget(Qt::LeftDockWidgetArea, dockSimulation);
    dockSimulation->setShown(false);

    connect(uiDockReadMap->btnInputMap, SIGNAL(clicked())
            , this, SLOT(loadReadingMap()));

    connect(uiDockReadMap->cmboMapCandidate, SIGNAL(currentIndexChanged(int))
            , this, SLOT(displayTheActivitedMap(int)));

    connect(uiDockReadMap->btnJump2Map, SIGNAL(clicked())
            , this, SLOT(jump2ReadingMapIndex()));

    connect(mapGView, SIGNAL(QGI_Node_clicked(QNode *))
            , this, SLOT(onQGI_NodeLeftClicked(QNode * )));

    connect(modeGroup, SIGNAL(triggered(QAction*))
            , this, SLOT(changeMode(QAction*)));

    connect(uiDockBuildMap->btnMakeNewNode, SIGNAL(clicked())
            , this, SLOT(buildModeNewNode()));

    connect(uiDockBuildMap->btnAddNodeIntoMap, SIGNAL(clicked())
            , this, SLOT(buildModeAddNode2MapView()));

    connect(uiDockBuildMap->btnDrawEdge, SIGNAL(toggled(bool))
            , mapGView, SLOT(switch2DrawEdgeMode(bool)));

    connect(mapGView, SIGNAL(newEdgeConnected(TopoEdge *))
            , this, SLOT(newEdgeConnected(TopoEdge *)));

    connect(uiDockBuildMap->btnSetEdgeOdom, SIGNAL(clicked())
            , this, SLOT(setEdgeOdom()));

    connect(uiDockBuildMap->btnSetRotation, SIGNAL(clicked())
            , this, SLOT(setNodeRotation()));

    connect(uiDockBuildMap->btnSaveMap, SIGNAL(clicked())
            , this, SLOT(saveBuiltMap()));

    connect(uiDockBuildMap->btnLoadMap, SIGNAL(clicked())
            , this, SLOT(loadBuiltMap()));

    connect(uiDockSimulation->btnConnectToROS, SIGNAL(clicked())
            , this, SLOT(initROS()));

    connect(mapGView, SIGNAL(rightClickOn_QGI_Node(QNode *))
            , this, SLOT(onQGI_NodeRightClicked(QNode *)));

    connect(uiDockBuildMap->cbNodesMovable, SIGNAL(toggled(bool))
            , this, SLOT(changeNodeMovable(bool)));

    connect(uiDockReadMap->cbMoveNode, SIGNAL(toggled(bool))
            , this, SLOT(changeNodeMovable(bool)));
}

TopoUI::~TopoUI()
{
    delete uiMain;
}

void TopoUI::loadReadingMap() {
    cleanReadDock(); //TODO

    QString name = uiDockReadMap->etInputMap->text();
    if (name.isEmpty()) {
        name = "map";
    }

    bool loadComplete;

    if (name.contains("real")) {
        /// we load the map from ROS
        initROS();
        if (!checkROS()) {
            setMsg("file name start with \"real\" means ask the realtime map, the following "
                   "number means the number of maps you would like to get, for example\n"
                   "real2 means you want the 2 real time map (with highest confidence");
        }
        name.remove(0, 3);
        auto mapNeeded = name.toUInt();
        topology_map::GetMapsRequest request;
        topology_map::GetMapsResponse response;
        request.requiredMaps = static_cast<unsigned short>(mapNeeded);
        if (srvC_askMaps.call(request, response)) {
            appendMsg("map loaded success");
            loadComplete = mapFromReading.readFromStr(response.mapJS);
        } else {
            loadComplete = false;
            appendMsg("service call fail");
        }
    } else {
        /// load the map from file
        loadComplete = loadMapGroupFromFile(name, mapFromReading);
    }


    if (loadComplete) {
        cout << "UI load map successful" << endl;

        setMsg("Total maps: " + QString::number(mapFromReading.getMapNumbers()) );

        int mapCounts = 0;

        for (const auto & mapCand: mapFromReading.getMapCollection().getMaps()) {
            QString comboInfo = QString("#%1 Confidence:%2")
                    .arg(mapCounts)
                    .arg(mapCand->getConfidence(
                            mapFromReading.getNodeCollection().experienceSize()));
            comboBoxMaps.push_back(mapCand);
            uiDockReadMap->cmboMapCandidate->addItem(comboInfo); // TODO use variant
            mapCounts++;
        }
    } else {
        appendMsg("UI load map FAIL");
        cerr << "UI load map FAIL" << endl;
    }
}

void TopoUI::displayTheActivitedMap(int index) {

    cleanTableView();

    MapCandidate & map2Draw = *comboBoxMaps[index];

    displayMapAtMapGV(map2Draw, true, false);
}

void TopoUI::saveBuiltMap() {
    QString name = uiDockBuildMap->etFileName->text();
    if (name.isEmpty()) {
        name = "built";
    }
    TopoFile topoFile{name.toStdString()};
    topoFile.open();
    topoFile.outputMap(mapFromBuilding);
}

void TopoUI::loadBuiltMap() {
    cleanTableView();
    QString name = uiDockBuildMap->etFileName->text();
    if (name.isEmpty()) {
        name = "built";
    }
    mapFromBuilding.selfClean();
    if (loadMapGroupFromFile(name, mapFromBuilding)) {
        const auto & mapCollection = mapFromBuilding.getMapCollection();
        if (mapCollection.mapNumbers() > 1) {
            bigBrother->setMsg("get more than 1 map candidate from built map, "
                               "ARE YOU SURE? Now we just display the first map");
        }
        displayMapAtMapGV(*mapCollection.getTheFirstMap(), true,
                          uiDockBuildMap->cbNodesMovable->isChecked());

    } else {
        setMsg("ERROR! CANT read the map:" + name);
    }
}

void TopoUI::onQGI_NodeLeftClicked(QNode *qgiNode) {
    if (CURRENT_MODE == READ_MODE) {
        nodeScene.clear();
        auto nodeJustForDisplay = new QNode(qgiNode->getRelatedNodeTOPO());
        nodeJustForDisplay->setDrawDetail(true);
        nodeScene.addItem(nodeJustForDisplay);
    }
    if (CURRENT_MODE == SIMULATION_MODE) {
        if (uiDockSimulation->btnPlaceRobot->isChecked()) {
            uiDockSimulation->btnPlaceRobot->setEnabled(false);
            uiDockSimulation->btnPlaceRobot->setChecked(false);
            robot = new QRobot(qgiNode);
            mapGView->scene()->addItem(robot);
            if (checkROS()) {
                pub_nodeInfo.publish(qgiNode->getRelatedNodeTOPO()->getInsCorrespond()
                                             ->encode2ROSmsg(0,0,0,0));
            }
        }
    }
}

void TopoUI::onQGI_NodeRightClicked(QNode * clickedNode) {
    if (CURRENT_MODE == SIMULATION_MODE) {
        int nums = clickedNode->getExitNums();
        if (robot != nullptr) {
            auto currentAt = robot->getCurrentAt();
            if (auto nodeWithRobot = dynamic_cast<QNode*>(currentAt)) {
                for (int i = 0; i < nums; i++) {
                    /// check if the robot is in the nearby node
                    if (nodeWithRobot == clickedNode->getQNodeAtExit(i)) {

                        const auto QEdgeMoved = clickedNode->getQEdgeAtExit(i);

                        /// send gate through msg
                        if (checkROS()) {
                            std_msgs::UInt8 tempGate;
                            tempGate.data = QEdgeMoved->getRelatedEdgeTOPO()
                                    ->getAnotherGate(clickedNode->getRelatedNodeTOPO());
                            pub_gateMove.publish(tempGate);
                        }

                        if (uiDockSimulation->cbNodeMoveDirectly->isChecked()) {
                            robot->move2(clickedNode);

                            /// send node msg
                            if (checkROS()) {
                                sendNodeROSmsg(clickedNode, QEdgeMoved, i);
                            }
                        } else {
                            robot->move2(QEdgeMoved);
                        }
                        return;
                    }
                }
            }
            else if (auto edgeWithRobot = dynamic_cast<QEdge*>(currentAt)) {
                for (int i = 0; i < nums; i++) {
                    if (edgeWithRobot == clickedNode->getQEdgeAtExit(i)) {
                        robot->move2(clickedNode);
                        if (checkROS()) {
                            sendNodeROSmsg(clickedNode, edgeWithRobot, i);
                        }
                    }
                }
            } else {
                cerr << "[TopoUI::onQGI_NodeRightClicked] ROBOT at nowhere!!!!" << endl;
            }
        }
    }
}

void TopoUI::sendNodeROSmsg(QNode *clickedNode, const QEdge *edgeWithRobot, int exit) {
    auto odomInfo = edgeWithRobot->getRelatedEdgeTOPO()
            ->getOdomData(clickedNode->getQNodeAtExit(exit)->getRelatedNodeTOPO());
    if (uiDockSimulation->cbEdgeNoise->isChecked()) {
        bool pass;
        double stdErr = uiDockSimulation->
                leEdgeNoise->text().toDouble(&pass);
        if (pass) {
            default_random_engine e(time(nullptr));
            normal_distribution<> n{0, stdErr};
            double dist = topo::calDis(odomInfo[0], odomInfo[1]);
            odomInfo[0] += n(e) * dist;
            odomInfo[1] += n(e) * dist;
        } else {
            setMsg("Please enter a right number");
        }
    }
    pub_nodeInfo.publish(clickedNode->getRelatedNodeTOPO()
                                 ->getInsCorrespond()->encode2ROSmsg(exit, odomInfo));
}

void TopoUI::changeMode(QAction * action) {
    dockBuildMap->setShown(false);
    dockReadMap->setShown(false);
    dockSimulation->setShown(false);

    infoView->clear();

    if (action == mode_READ) {
        cout << "switch to read mode" << endl;
        CURRENT_MODE = READ_MODE;
        dockReadMap->setShown(true);
        cleanTableView();
    } else {

    }

    if (action == mode_BUILD) {
        cout << "switch to build mode" << endl;
        CURRENT_MODE = BUILD_MODE;
        dockBuildMap->setShown(true);
        uiDockBuildMap->btnMakeNewNode->setText("make a new node");
        cleanTableView();
    } else {
        uiDockBuildMap->btnDrawEdge->setChecked(false);
        uiDockBuildMap->cbNodesMovable->setChecked(false);
    }

    if (action == mode_SIMULATION) {
        cout << "switch to simulation mode" << endl;
        CURRENT_MODE = SIMULATION_MODE;
        dockSimulation->setShown(true);
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
//            mapFromBuilding.addInstanceDirectly(pIns);
            uiDockBuildMap->cmboMadeNodes->addItem(nameStr, thePtrVariant);
            uiDockBuildMap->cmboMadeNodes->setCurrentIndex(
                    uiDockBuildMap->cmboMadeNodes->count() - 1);
        }
        uiDockBuildMap->btnMakeNewNode->setText("make a new node");
        uiDockBuildMap->cmboMadeNodes->setEnabled(true);
        uiDockBuildMap->btnAddNodeIntoMap->setEnabled(true);
    }

}

void TopoUI::buildModeAddNode2MapView() {
    auto pBox = uiDockBuildMap->cmboMadeNodes;
    NodeInstance * sampleIns =
            static_cast<NodeInstance *>(pBox->itemData(pBox->currentIndex()).value<void*>());
    auto * pIns = new NodeInstance(*sampleIns);
    auto pnode = new TopoNode(pIns);
    mapFromBuilding.addTopoNodeDirectly(pnode);
    mapFromBuilding.addInstanceDirectly(pIns);
    auto QGI_node = new QNode(pnode);
    QGI_node->setDrawDetail(true);
    QGI_node->setFlag(QGraphicsItem::ItemIsMovable);
//    QGI_node->setAcceptDrops(true);
    mapGView->scene()->addItem(QGI_node);
}

void TopoUI::newEdgeConnected(TopoEdge * newEdge) {
    mapFromBuilding.getMapCollection().getTheFirstMap()->addEdgeDirectly(newEdge);
}

void TopoUI::setMsg(const QString & msg) {
    infoView->setText(msg);
}

void TopoUI::appendMsg(const QString & msg) {
    infoView->append("\n" + msg);
}

void TopoUI::setEdgeOdom() {
    bool pass = false;
    const auto & str = uiDockBuildMap->leEdgeOdom->text();
    auto strs = str.split(',');
    if (strs.size() == 2) {
        double odomX = strs[0].toDouble(&pass);
        if (pass) {
            double odomY = strs[1].toDouble(&pass);
            if (pass) {
                const auto & list = mapGView->scene()->selectedItems();
                if (!list.isEmpty()) {
                    uiDockBuildMap->cbNodesMovable->setChecked(false);
                    for (auto & item: list) {
                        if (auto edgeItem = dynamic_cast<QEdge*> (item)) {
                            edgeItem->getRelatedEdgeTOPO()->
                                    setOdomDataDirectly(odomX, odomY, 0);
                        }
                    }
                    displayMapAtMapGV(*mapFromBuilding.getMapCollection().getTheFirstMap(),
                                      true, uiDockBuildMap->cbNodesMovable->isChecked());
                }
//                else {
//                    for (auto & item: mapGView->scene()->items()) {
//                        if (auto edgeItem = dynamic_cast<QEdge*> (item)) {
//                            edgeItem->getRelatedEdgeTOPO()->
//                                    setOdomDataDirectly(odomX, odomY, 0);
//                        }
//                    }
//                }

                return;
            }
        }
    }

    bigBrother->setMsg("Please enter a correct odom\n"
                       "Example: -5.4,3");
}

void TopoUI::setNodeRotation() {
    bool pass = false;
    double rotation = uiDockBuildMap->leRotation->text().toInt(&pass);

    if (pass) {
        const auto & list = mapGView->scene()->selectedItems();
        if (!list.isEmpty()) {
            for (auto & item: list) {
                if (auto nodeItem = dynamic_cast<QNode*> (item)) {
                    nodeItem->setRotation(-rotation);
                }
            }
        }
//        else {
//            for (auto & item: mapGView->scene()->items()) {
//                if (auto edgeItem = dynamic_cast<QEdge*> (item)) {
//                    edgeItem->setLength(rotation);
//                }
//            }
//        }
    } else {
        bigBrother->setMsg("Please enter a correct number");
    }
}


void TopoUI::displayMapAtMapGV(MapCandidate & map2Draw, bool detailed, bool movable) {
    mapGView->scene()->clear();
    map2Draw.cleanAllNodeFlagsAndPtr();

    auto beginNode = map2Draw.getOneTopoNode();

    queue<TopoNode*> lookupQueue;

    auto nodeQGI = new QNode(beginNode);
    beginNode->setAssistPtr(nodeQGI);
    mapScene.addItem(nodeQGI);
    if (detailed) {
        nodeQGI->setDrawDetail(true);
    }
    if (movable) {
        nodeQGI->setFlag(QGraphicsItem::ItemIsMovable);
    }
    lookupQueue.push(beginNode);

    while(!lookupQueue.empty()) {
        auto & curNode = lookupQueue.front();
        lookupQueue.pop();
        auto * curQNode = static_cast<QNode*>(curNode->getAssistPtr());
        QPointF curPos = curQNode->pos();

        //look into every edge
        for (gateId edgeNo = 0; edgeNo < curNode->getInsCorrespond()->sizeOfExits(); edgeNo++) {

            //means this edge have been drawn
            if (curQNode->getQEdgeAtExit(edgeNo) != nullptr) {
                continue;
            }

            auto curEdge = curNode->getEdge(edgeNo);
            //means this edge has never been moved through
            if (curEdge == nullptr) {
                continue;
            }


            TopoNode * anotherNode = curEdge->getAnotherNode(curNode);
            uint8_t anotherGate = curEdge->getAnotherGate(curNode);

            //draw the other node
            if (anotherNode->getAssistPtr() == nullptr) {
                auto odomData = curEdge->getOdomData(curNode);  //todo
                QPointF dist{odomData[0], -odomData[1]};
                //ENU is different with the UI coor
                const auto & exitOfAnotherNode =
                        anotherNode->getInsCorrespond()->getExits()[anotherGate];
                QPointF disInAnotherNode{ exitOfAnotherNode.getPosX(),
                                         -exitOfAnotherNode.getPosY()};
                dist -= disInAnotherNode;
                const auto & exitOfThisNode =
                        curNode->getInsCorrespond()->getExits()[edgeNo];
                dist += {exitOfThisNode.getPosX(), -exitOfThisNode.getPosY()};
                auto anotherQNode = new QNode(anotherNode);
                anotherNode->setAssistPtr(anotherQNode);
                anotherQNode->setPos(curPos + dist * METER_TO_PIXLE);
                mapScene.addItem(anotherQNode);
                if (detailed) {
                    anotherQNode->setDrawDetail(true);
                }
                if (movable) {
                    anotherQNode->setFlag(QGraphicsItem::ItemIsMovable);
                }
            }

            //draw the edge between them
            auto edge2Draw = new QEdge(curEdge, curQNode,
                                       static_cast<QNode *>(anotherNode->getAssistPtr()));
            mapScene.addItem(edge2Draw);

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
//        qDebug() << tempStr << ", " << static_cast<QNode*>(node->getAssistPtr())->pos();
    }
}

void TopoUI::cleanTableView() {
    mapScene.clear();
    nodeScene.clear();
    infoView->clear();
}

void TopoUI::cleanReadDock() {
    mapFromReading.selfClean();
    uiDockReadMap->cmboMapCandidate->clear();   //TODO use QVariant
    comboBoxMaps.clear();   //TODO WARNNING leak
}

bool TopoUI::loadMapGroupFromFile(const QString & fileName, MapArranger & dist) {
    return dist.reloadFromFile(fileName.toStdString());
}

void TopoUI::initROS() {
    if (checkROS()) {
        infoView->setText("you have connected to ROS");
        return;
    }
    int argc = 0;
    char ** argv = nullptr;
    ros::init(argc, argv, "TopoUINode");
    if (!checkROS()) {
        infoView->setText("CANT find the ROS master, plz run roscore and try again");
        return;
    }
    infoView->setText("ROS connect success!");

    uiDockSimulation->btnConnectToROS->setChecked(true);
    uiDockSimulation->btnConnectToROS->setEnabled(false);

    ros::start();
    ros::NodeHandle n;

    pub_nodeInfo = n.advertise<topology_map::NewNodeMsg>(
            TOPO_STD_TOPIC_NAME_NODEINFO, 0);
    pub_gateMove = n.advertise<std_msgs::UInt8>(
            TOPO_STD_TOPIC_NAME_GATEMOVE, 0);
    srvC_askMaps = n.serviceClient<topology_map::GetMaps>(TOPO_STD_SERVICE_NAME_GETMAPS);
}

void TopoUI::changeNodeMovable(bool movable) {
    if (CURRENT_MODE == READ_MODE) {
        displayTheActivitedMap(uiDockReadMap->cmboMapCandidate->currentIndex());
    }
    for (auto & item : mapGView->scene()->items()) {
        if (auto nodeItem = dynamic_cast<QNode*>(item)) {
            nodeItem->setFlag(QGraphicsItem::ItemIsMovable, movable);
        }
    }
}

bool TopoUI::checkROS() {
    return ros::master::check();
}

void TopoUI::jump2ReadingMapIndex() {
    int index = uiDockReadMap->leMapID->text().toUInt();
    int maxNum = uiDockReadMap->cmboMapCandidate->count();
    if (index >= maxNum) {
        setMsg("please enter a correct number\n"
               "There are " + QString::number(maxNum) + " maps\n(Start from 0)");
        return;
    }
    uiDockReadMap->cmboMapCandidate->setCurrentIndex(index);
}


