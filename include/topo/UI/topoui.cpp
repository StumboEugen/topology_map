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
#include "ui_dockrealtime.h"
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
#include <topology_map/LeaveNode.h>
#include <topology_map/NewNodeMsg.h>
#include <topology_map/PathPlanning.h>

using namespace std;

UIMode CURRENT_MODE = READ_MODE;
TopoUI * bigBrother;

TopoUI::TopoUI(QWidget *parent) :
    QMainWindow(parent),
    uiMain(new Ui::TopoUI),
    uiDockReadMap(new Ui::DockReadMapUI),
    uiDockBuildMap(new Ui::DockBuildMapUI),
    uiDockSimulation(new Ui::DockSimulationUI),
    uiDockRealTime(new Ui::DockRealTimeUI)
{
    bigBrother = this;

    uiMain->setupUi(this);
    setWindowTitle("TOPO Viewer");

    modeGroup = new QActionGroup(this);

    mode_READ = modeGroup->addAction("read file mode");
    mode_BUILD = modeGroup->addAction("build map mode");
    mode_SIMULATION = modeGroup->addAction("simulation mode");
    mode_REALTIME = modeGroup->addAction("realtime mode");

    mode_READ->setCheckable(true);
    mode_BUILD->setCheckable(true);
    mode_SIMULATION->setCheckable(true);
    mode_REALTIME->setCheckable(true);

    mode_READ->setChecked(true);

    uiMain->mainToolBar->addAction(mode_READ);
    uiMain->mainToolBar->addAction(mode_BUILD);
    uiMain->mainToolBar->addAction(mode_SIMULATION);
    uiMain->mainToolBar->addAction(mode_REALTIME);
    uiMain->mainToolBar->addSeparator();

    QAction * dragMode = new QAction("Drag Mode", uiMain->mainToolBar);
    dragMode->setCheckable(true);
    uiMain->mainToolBar->addAction(dragMode);
    uiMain->mainToolBar->addSeparator();
    connect(dragMode, SIGNAL(toggled(bool)), this, SLOT(setMapGViewDragMode(bool)));

    qactConnectToROS = new QAction("Connect to ROS", uiMain->mainToolBar);
    qactConnectToROS->setCheckable(true);
    uiMain->mainToolBar->addAction(qactConnectToROS);

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

    dockRealTime = initTheDock("DockRealTime");
    uiDockRealTime->setupUi(dockRealTime);
    addDockWidget(Qt::LeftDockWidgetArea, dockRealTime);
    dockRealTime->setShown(false);

    connect(uiDockReadMap->btnInputMap, SIGNAL(clicked())
            , this, SLOT(loadReadingMap()));

    connect(uiDockReadMap->cmboMapCandidate, SIGNAL(currentIndexChanged(int))
            , this, SLOT(displayCandidateFromReading(int)));

    connect(uiDockReadMap->btnJump2Map, SIGNAL(clicked())
            , this, SLOT(jump2ReadingMapIndex()));

    connect(uiDockReadMap->cbMoveNode, SIGNAL(toggled(bool))
            , this, SLOT(changeNodeMovable(bool)));

    connect(mapGView, SIGNAL(QGI_Node_clicked(QNode *))
            , this, SLOT(onQGI_NodeLeftClicked(QNode * )));

    connect(mapGView, SIGNAL(rightClickOn_QGI_Node(QNode *))
            , this, SLOT(onQGI_NodeRightClicked(QNode *)));

    connect(mapGView, SIGNAL(newEdgeConnected(TopoEdge *))
            , this, SLOT(newEdgeConnected(TopoEdge *)));

    connect(modeGroup, SIGNAL(triggered(QAction*))
            , this, SLOT(changeMode(QAction*)));

    connect(uiDockBuildMap->btnMakeNewNode, SIGNAL(clicked())
            , this, SLOT(buildModeNewNode()));

    connect(uiDockBuildMap->btnAddNodeIntoMap, SIGNAL(clicked())
            , this, SLOT(buildModeAddNode2MapView()));

    connect(uiDockBuildMap->btnDrawEdge, SIGNAL(toggled(bool))
            , mapGView, SLOT(switch2DrawEdgeMode(bool)));

    connect(uiDockBuildMap->btnSetEdgeOdom, SIGNAL(clicked())
            , this, SLOT(setEdgeOdom()));

    connect(uiDockBuildMap->btnSetRotation, SIGNAL(clicked())
            , this, SLOT(setNodeRotation()));

    connect(uiDockBuildMap->btnSaveMap, SIGNAL(clicked())
            , this, SLOT(saveBuiltMap()));

    connect(uiDockBuildMap->btnLoadMap, SIGNAL(clicked())
            , this, SLOT(loadBuiltMap()));

    connect(uiDockBuildMap->cbNodesMovable, SIGNAL(toggled(bool))
            , this, SLOT(changeNodeMovable(bool)));

    connect(qactConnectToROS, SIGNAL(changed())
            , this, SLOT(initROS()));

    connect(uiDockRealTime->btnGetRealTimeMap, SIGNAL(clicked())
            , this, SLOT(askForRealTimeMap()));

    connect(uiDockRealTime->cbCandidates, SIGNAL(currentIndexChanged(int))
            , this, SLOT(displayCandidateFromRealTime(int)));
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
        name.remove(0, 4);
        cout << name.toStdString() << endl;
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

        mapFromReading.sortByConfidence();

        int mapCounts = 0;

        auto & mapCollection = mapFromReading.getMapCollection();
        mapCollection.calSumOfConfidence();
        for (const auto & mapCand: mapCollection.getMaps()) {
            QString comboInfo = QString("#%1 Confidence:%2")
                    .arg(mapCounts)
                    .arg(mapCand->getConfidence(
                            mapFromReading.getNodeCollection().experienceSize())
                         / mapCollection.getSumOfConfidence());
            comboBoxMaps.push_back(mapCand);
            uiDockReadMap->cmboMapCandidate->addItem(comboInfo); // TODO use variant
            mapCounts++;
        }
    } else {
        appendMsg("UI load map FAIL");
        cerr << "UI load map FAIL" << endl;
    }
}

void TopoUI::displayCandidateFromReading(int index) {

    if (index < 0 || index >= comboBoxMaps.size()) {
        return;
    }

    cleanTableView();

    MapCandidate & map2Draw = *comboBoxMaps[index];

    displayMapAtMapGV(map2Draw, true, true, false);
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
        displayMapAtMapGV(*mapCollection.getTheFirstMap(), false, true,
                          uiDockBuildMap->cbNodesMovable->isChecked(),
                          true);

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
            } else {
                setMsg("Please connect to ROS first!");
            }
        }
    }
}

void TopoUI::onQGI_NodeRightClicked(QNode * clickedNode) {
    if (CURRENT_MODE == SIMULATION_MODE) {
        if (!uiDockSimulation->cbPathPlanning->isChecked())
        {
            int nums = clickedNode->getExitNums();
            if (robot != nullptr)
            {
                auto currentAt = robot->getCurrentAt();
                /// check if the robot currently is on edge or node
                if (auto nodeWithRobot = dynamic_cast<QNode*>(currentAt))
                {
                    for (int i = 0; i < nums; i++)
                    {
                        /// check if the robot is in the nearby node
                        if (nodeWithRobot == clickedNode->getQNodeAtExit(i))
                        {

                            // the edge between the click node and the robot's node
                            const auto QEdgeMoved = clickedNode->getQEdgeAtExit(i);

                            /// send gate through msg
                            if (checkROS())
                            {
                                topology_map::LeaveNode nodeLeavingMsg;
                                // the left gate in ground truth
                                const auto leftGate = QEdgeMoved->getRelatedEdgeTOPO()
                                        ->getAnotherGate(clickedNode->getRelatedNodeTOPO());
                                nodeLeavingMsg.leaveGate = leftGate;
                                nodeLeavingMsg.leaveDir =
                                        (float) nodeWithRobot->getRelatedNodeTOPO()
                                                ->getInsCorrespond()->getExits()
                                                .at(nodeLeavingMsg.leaveGate).getMidRad();

                                if (instanceWithNoise)
                                {
                                    // means that the exit noise is added
                                    const auto& theLeftNode =
                                            QEdgeMoved->getRelatedEdgeTOPO()->getAnotherNode
                                                    (clickedNode->getRelatedNodeTOPO());
                                    const auto& theLeftExit =
                                            theLeftNode->getInsCorrespond()
                                            ->getExits()[leftGate];
                                    nodeLeavingMsg.leaveGate =
                                            instanceWithNoise->figureOutWhichExitItis(
                                                    theLeftExit.getPosX(),
                                                    theLeftExit.getPosY());
                                    pub_gateMove.publish(nodeLeavingMsg);
                                }
                                else
                                {
                                    pub_gateMove.publish(nodeLeavingMsg);
                                }

                                if (uiDockSimulation->cbNodeMoveDirectly->isChecked())
                                {
                                    ros::Duration(0.1).sleep();
                                    robot->move2(clickedNode);

                                    /// send node msg
                                    if (checkROS())
                                    {
                                        sendNodeROSmsg(clickedNode, QEdgeMoved, i);
                                    }
                                }
                                else
                                {
                                    robot->move2(QEdgeMoved);
                                }
                            }
                            return;
                        }
                    }
                }
                else if (auto edgeWithRobot = dynamic_cast<QEdge*>(currentAt))
                {
                    for (int i = 0; i < nums; i++)
                    {
                        if (edgeWithRobot == clickedNode->getQEdgeAtExit(i))
                        {
                            robot->move2(clickedNode);
                            if (checkROS())
                            {
                                sendNodeROSmsg(clickedNode, edgeWithRobot, i);
                            }
                        }
                    }
                }
                else
                {
                    cerr << "[TopoUI::onQGI_NodeRightClicked] ROBOT at nowhere!!!!" << endl;
                }
            }
        }
        else
        {
            auto & mapCollection = mapFromBuilding.getMapCollection();
            auto & path = mapCollection.getTopoPath();
            auto robotPlace = dynamic_cast<QNode*>(robot->getCurrentAt());
            if (robotPlace == nullptr) {
                setMsg("plan failure:\nrobot is not at node");
                return;
            }
            path.findPath(currentDrawnMap,
                          robotPlace->getRelatedNodeTOPO()->getInsCorrespond(),
                          clickedNode->getRelatedNodeTOPO()->getInsCorrespond());
            QNode* currentQNode = robotPlace;
            for (const auto & step : path.getPath()) {
                currentQNode->setSelected(true);
                auto gateId = step.beginNode->gateIdOfTheTopoEdge(step.stepEdge);
                QEdge* nextEdge = currentQNode->getQEdgeAtExit(gateId);
                nextEdge->setSelected(true);
                currentQNode = nextEdge->getAnotherNode(currentQNode);
            }
            clickedNode->setSelected(true);
        }
    }
    else if (CURRENT_MODE == REALTIME_MODE)
    {
        if (uiDockRealTime->cbEnablePathPlanning->isChecked())
        {
            auto & mapCollection = mapFromRealTime.getMapCollection();
            auto & path = mapCollection.getTopoPath();
            auto robotPlace = dynamic_cast<QNode*>(robot->getCurrentAt());
            if (robotPlace == nullptr) {
                setMsg("plan failure:\nrobot is not at node");
                return;
            }
            path.findPath(currentDrawnMap,
                    robotPlace->getRelatedNodeTOPO()->getInsCorrespond(),
                    clickedNode->getRelatedNodeTOPO()->getInsCorrespond());
            QNode* currentQNode = robotPlace;
            for (const auto & step : path.getPath()) {
                currentQNode->setSelected(true);
                auto gateId = step.beginNode->gateIdOfTheTopoEdge(step.stepEdge);
                QEdge* nextEdge = currentQNode->getQEdgeAtExit(gateId);
                nextEdge->setSelected(true);
                currentQNode = nextEdge->getAnotherNode(currentQNode);
            }
            clickedNode->setSelected(true);
        }

        if (uiDockRealTime->cbSendPlanningReq->isChecked())
        {
            auto robotPlace = dynamic_cast<QNode*>(robot->getCurrentAt());
            if (robotPlace == nullptr) {
                setMsg("plan req failure:\nrobot is not at node");
                return;
            }
            topology_map::PathPlanningRequest req;
            topology_map::PathPlanningResponse res;
            req.beginInsSerialN = robotPlace->getRelatedNodeTOPO()
                    ->getInsCorrespond()->getSerialNumber();
            req.goalInsSerialN = clickedNode->getRelatedNodeTOPO()
                    ->getInsCorrespond()->getSerialNumber();
            req.rankOfPlanningMap = uiDockRealTime->cbCandidates->currentIndex();
            if (srvC_pathPlanning.call(req, res))
            {
                QNode* currentQNode = robotPlace;
                currentQNode->setSelected(true);

                const auto & steps = res.pathInsSerialN;
                for (size_t nStep = 1; nStep < steps.size(); ++nStep)
                {
                    for (int nExit = 0; nExit < currentQNode->getExitNums(); ++nExit) {
                        QNode* anotherQNode = currentQNode->getQNodeAtExit(nExit);
                        if (anotherQNode == nullptr) {
                            continue;
                        }
                        auto serial = anotherQNode->getRelatedNodeTOPO()
                                ->getInsCorrespond()->getSerialNumber();
                        if (serial == steps[nStep]) {
                            currentQNode->getQEdgeAtExit(nExit)->setSelected(true);
                            anotherQNode->setSelected(true);
                            currentQNode = anotherQNode;
                        }
                    }
                }
            } else {
                setMsg("service call fail!");
            }
        }
    }
}

void TopoUI::sendNodeROSmsg(QNode *clickedNode, const QEdge *edgeWithRobot, int exitGate) {
    auto odomInfo = edgeWithRobot->getRelatedEdgeTOPO()
            ->getOdomData(clickedNode->getQNodeAtExit(exitGate)->getRelatedNodeTOPO());
    //TODO arrange this better
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
            setMsg("Please enter a right number in the edge odom noise");
        }
    }

    topology_map::NewNodeMsg newNodeMsg;
    newNodeMsg.odomX = static_cast<float>(odomInfo[0]);
    newNodeMsg.odomY = static_cast<float>(odomInfo[1]);


    const auto & theRelatedIns = clickedNode->getRelatedNodeTOPO()->getInsCorrespond();
    newNodeMsg.exitNum = theRelatedIns->sizeOfExits();

    if (instanceWithNoise) {
        delete instanceWithNoise;
        instanceWithNoise = nullptr;
    }

    if (uiDockSimulation->cbExitNoise->isChecked()) {
        bool pass;
        double range = uiDockSimulation->leNodeNoise->text().toDouble(&pass);
        if (pass) {
            default_random_engine e(time(nullptr));
            uniform_real_distribution<> u(-range, range);

            static bool fakeB = false;

            /// for the correct arrive at, we need to build a new nodeInstance
            instanceWithNoise = new NodeInstance(false);
            for (const auto & exitIns: theRelatedIns->getExits()) {

//                { //TODO DELETE THIS FOR DEBUG
//                    double e1 = fakeB ? 0.05 : -0.05;
//                    double e2 = fakeB ? 0.05 : -0.05;
//                    fakeB = !fakeB;
//
//                    instanceWithNoise->addExit(
//                            exitIns.getPosX() + e1,
//                            exitIns.getPosY() + e2,
//                            exitIns.getOutDir());
//                }

                instanceWithNoise->addExit(
                        exitIns.getPosX() + u(e),
                        exitIns.getPosY() + u(e),
                        exitIns.getOutDir());
            }
            instanceWithNoise->completeAdding();

            const auto & theExitInsInGroundTruth = theRelatedIns->getExits()[exitGate];
            newNodeMsg.arriveAt = instanceWithNoise->figureOutWhichExitItis(
                    theExitInsInGroundTruth.getPosX(),
                    theExitInsInGroundTruth.getPosY());

            for (const auto & exitIns: instanceWithNoise->getExits()) {
                newNodeMsg.midPosXs.push_back((float &&) exitIns.getPosX());
                newNodeMsg.midPosYs.push_back((float &&) exitIns.getPosY());
                newNodeMsg.outDirs.push_back((float &&) exitIns.getOutDir());
            }

        } else {
            setMsg("Please enter a right number in the node exit noise");
        }
    } else {
        for (const auto & exitIns: theRelatedIns->getExits()) {
            newNodeMsg.midPosXs.push_back((float &&) exitIns.getPosX());
            newNodeMsg.midPosYs.push_back((float &&) exitIns.getPosY());
            newNodeMsg.outDirs.push_back((float &&) exitIns.getOutDir());
        }
        newNodeMsg.arriveAt = static_cast<unsigned short>(exitGate);
    }

    pub_nodeInfo.publish(newNodeMsg);
}

void TopoUI::changeMode(QAction * action) {
    static QAction * lastAction = mode_READ;

    if (action == lastAction) {
        return;
    }

    dockBuildMap->setShown(false);
    dockReadMap->setShown(false);
    dockSimulation->setShown(false);
    dockRealTime->setShown(false);

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
        if (lastAction != mode_BUILD) {
            cleanTableView();
        }
    }

    if (action == mode_REALTIME) {
        cout << "switch to simulation mode" << endl;
        CURRENT_MODE = REALTIME_MODE;
        dockRealTime->setShown(true);
        cleanTableView();
    }

    lastAction = action;
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

    uiDockBuildMap->cbNodesMovable->setChecked(true);

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
                                      false, true, uiDockBuildMap->cbNodesMovable->isChecked());
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


void TopoUI::displayMapAtMapGV(MapCandidate & map2Draw,
                               bool drawRobot,
                               bool detailed,
                               bool movable,
                               bool fillInsOdom) {
    mapGView->scene()->clear();
    map2Draw.cleanAllNodeFlagsAndPtr();

    currentDrawnMap = &map2Draw;

    const auto & robotPlace = map2Draw.getCurrentNode();

    auto beginNode = map2Draw.getOneTopoNode();
    if (fillInsOdom)
    {
        for (auto & ins: beginNode->getRelatedInses()) {
            ins->setGlobalPos(0.0, 0.0, 0.0);
        }
    }

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

    /// BFS
    while(!lookupQueue.empty()) {
        auto & curNode = lookupQueue.front();
        lookupQueue.pop();
        auto * curQNode = static_cast<QNode*>(curNode->getAssistPtr());
        QPointF curPos = curQNode->pos();

        if (curNode == robotPlace && drawRobot) {
            robot = new QRobot(curQNode);
            mapScene.addItem(robot);
            if (CURRENT_MODE == REALTIME_MODE) {
                nodeScene.clear();
                auto nodeForCmd = new QNode(curNode);
                nodeForCmd->setRealTimeMode(true);
                nodeForCmd->setDrawDetail(true);
                nodeScene.addItem(nodeForCmd);
            }
        }

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
//                const auto & exitOfAnotherNode =
//                        anotherNode->getInsCorrespond()->getExits()[anotherGate];
//                QPointF disInAnotherNode{ exitOfAnotherNode.getPosX(),
//                                         -exitOfAnotherNode.getPosY()};
//                dist -= disInAnotherNode;
//                const auto & exitOfThisNode =
//                        curNode->getInsCorrespond()->getExits()[edgeNo];
//                dist += {exitOfThisNode.getPosX(), -exitOfThisNode.getPosY()};
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
                if (fillInsOdom) {
                    for (auto & ins : anotherNode->getRelatedInses()) {
                        NodeInstance* curIns = curNode->getInsCorrespond();
                        ins->setGlobalPos(curIns->getGlobalX() + odomData[0],
                                          curIns->getGlobalY() + odomData[1],
                                          0.0);
                    }
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
        infoView->setText("connect to ROS successfully");
        return;
    }
    int argc = 0;
    char ** argv = nullptr;
    ros::init(argc, argv, "TopoUINode", ros::init_options::AnonymousName);
    if (!checkROS()) {
        infoView->setText("CANT find the ROS master, plz run roscore and try again");
        return;
    }
    infoView->setText("ROS connect success!");

    qactConnectToROS->setChecked(true);
    qactConnectToROS->setEnabled(false);

    ros::start();
    ros::NodeHandle n;

    pub_nodeInfo = n.advertise<topology_map::NewNodeMsg>(
            TOPO_STD_TOPIC_NAME_NODEINFO, 0);
    pub_gateMove = n.advertise<topology_map::LeaveNode>(
            TOPO_STD_TOPIC_NAME_GATEMOVE, 0);
    srvC_askMaps = n.serviceClient<topology_map::GetMaps>(TOPO_STD_SERVICE_NAME_GETMAPS);
    srvC_pathPlanning =n.serviceClient<topology_map::PathPlanning>
            (TOPO_STD_SERVICE_NAME_PATHPLANNING);
}

void TopoUI::changeNodeMovable(bool movable) {
    if (CURRENT_MODE == READ_MODE) {
        displayCandidateFromReading(uiDockReadMap->cmboMapCandidate->currentIndex());
    }
    for (auto & item : mapGView->scene()->items()) {
        if (auto nodeItem = dynamic_cast<QNode*>(item)) {
            nodeItem->setFlag(QGraphicsItem::ItemIsMovable, movable);
        }
    }
}

bool TopoUI::checkROS() {
    return ros::isInitialized();
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

void TopoUI::askForRealTimeMap() {
    if (!checkROS()) {
        setMsg("Please connect to ROS first");
        return;
    }

    auto mapNeeded = uiDockRealTime->sbMapNeeded->text().toUInt();
    topology_map::GetMapsRequest request;
    topology_map::GetMapsResponse response;
    request.requiredMaps = static_cast<unsigned short>(mapNeeded);
    if (srvC_askMaps.call(request, response)) {
        appendMsg("real time map loaded success");

        uiDockRealTime->cbCandidates->clear();

        mapFromRealTime.readFromStr(response.mapJS);

        int mapCounts = 0;
        auto & mapCollection = mapFromRealTime.getMapCollection();
        mapCollection.calSumOfConfidence();
        for (auto mapCand : mapCollection.getOrderedMaps()) {
            double confidence = mapCand->getConfidence(
                                        mapFromRealTime.getNodeCollection().experienceSize());
            confidence /= mapCollection.getSumOfConfidence();
            QString comboInfo = QString("#%1 Confidence:%2")
                    .arg(mapCounts++)
                    .arg(confidence);
            uiDockRealTime->cbCandidates->addItem(comboInfo);
        }
    } else {
        appendMsg("service call fail");
    }
}

void TopoUI::displayCandidateFromRealTime(int index) {
    if (index < 0) {
        return;
    }

    cleanTableView();
    const auto & maps = mapFromRealTime.getMapCollection().getOrderedMaps();
    if (index >= maps.size()) {
        return;
    }
    
    bool fillInsOdom = index == 0;
    displayMapAtMapGV(*maps[index], true,
            true, false, fillInsOdom);
}

void TopoUI::realTimeMode_sendMoveCmd(int exit) {
    auto currentQNode = dynamic_cast<QNode*>(robot->getCurrentAt());
    if (currentQNode != nullptr) {
        if (!checkROS()) {
            appendMsg("ROS connection FAIL!");
            return;
        }

        topology_map::LeaveNode msg;
        msg.leaveGate = static_cast<unsigned short>(exit);
        msg.leaveDir = (float) currentQNode->getRelatedNodeTOPO()
                        ->getInsCorrespond()->getExits()[exit].getMidRad();
        pub_gateMove.publish(msg);

        nodeGView->scene()->clear();
        robot->move2(currentQNode->getQEdgeAtExit(exit));

        appendMsg("pub Success, wait for the next approach to ask for new map");
    }
}

void TopoUI::keyPressEvent(QKeyEvent *event) {
    QWidget::keyPressEvent(event);

    if (CURRENT_MODE == REALTIME_MODE && (event->modifiers() & Qt::ControlModifier)) {
        switch (event->key()) {
            case Qt::Key_R:
                uiDockRealTime->btnGetRealTimeMap->click();
                break;

            case Qt::Key_0:
                realTimeMode_sendMoveCmd(0);
                break;

            case Qt::Key_1:
                realTimeMode_sendMoveCmd(1);
                break;

            case Qt::Key_2:
                realTimeMode_sendMoveCmd(2);
                break;

            case Qt::Key_3:
                realTimeMode_sendMoveCmd(3);
                break;

            case Qt::Key_4:
                realTimeMode_sendMoveCmd(4);
                break;


            default:break;
        }
    }
}


