#ifndef TOPOUI_H
#define TOPOUI_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QActionGroup>
#include <QDockWidget>
#include <QTextEdit>
#include <QTextBrowser>

#include "topo/Topo.h"
#include "TopoMapGView.h"
#include "TopoNodeGView.h"
#include "UITOOLS.h"
#include "QRobot.h"

#include <vector>
#include <iostream>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <topology_map/GetMaps.h>

namespace Ui {
    class TopoUI;
    class DockReadMapUI;
    class DockBuildMapUI;
    class DockSimulationUI;
    class DockRealTimeUI;
}

class TopoUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit TopoUI(QWidget *parent = nullptr);
    ~TopoUI() override;

    void appendMsg(const QString & msg);
    void setMsg(const QString & msg);

    void realTimeMode_sendMoveCmd(int);

protected:
    void keyPressEvent(QKeyEvent *event) override;

private:

    Ui::TopoUI *uiMain;
    QActionGroup * modeGroup;
    QAction * mode_READ;
    QAction * mode_BUILD;
    QAction * mode_SIMULATION;
    QAction * mode_REALTIME;

    QAction * qactConnectToROS;

    QRobot * robot;

    ros::Publisher pub_nodeInfo;
    ros::Publisher pub_gateMove;
    ros::ServiceClient srvC_askMaps;
    ros::ServiceClient srvC_pathPlanning;

    bool checkROS();

    MapArranger mapFromReading;
    MapArranger mapFromBuilding;
    MapArranger mapFromRealTime;

    MapCandidate * currentDrawnMap;

    QHBoxLayout * centerLayout;

    TopoMapGView * mapGView;
    QGraphicsScene mapScene;

    QVBoxLayout * smallWindowLayout;

    QTextBrowser * infoView;
    TopoNodeGView * nodeGView;
    QGraphicsScene nodeScene;

    Ui::DockReadMapUI * uiDockReadMap;
    QDockWidget * dockReadMap;
    std::vector<MapCandidate*> comboBoxMaps;

    Ui::DockBuildMapUI * uiDockBuildMap;
    QDockWidget * dockBuildMap;
    std::vector<TopoNode *> buildModeNodes;

    Ui::DockSimulationUI * uiDockSimulation;
    QDockWidget * dockSimulation;

    Ui::DockRealTimeUI * uiDockRealTime;
    QDockWidget * dockRealTime;

    void cleanTableView();
    void cleanReadDock();

    QDockWidget * initTheDock(const char *objectName);

    void displayMapAtMapGV(MapCandidate &,
            bool drawRobot = false,
            bool detailed = true,
            bool movable = false,
            bool fillInsOdom = false);

    bool loadMapGroupFromFile(const QString & fileName, MapArranger & dist);

    void sendNodeROSmsg(QNode *clickedNode, const QEdge *edgeWithRobot, int exitGate);

    NodeInstance * instanceWithNoise = nullptr;

private Q_SLOTS:
    void loadReadingMap();
    void displayCandidateFromReading(int);
    void jump2ReadingMapIndex();
    void onQGI_NodeLeftClicked(QNode *);
    void onQGI_NodeRightClicked(QNode *);
    void changeMode(QAction *);
    void setMapGViewDragMode(bool);
    void buildModeNewNode();
    void buildModeAddNode2MapView();
    void newEdgeConnected(TopoEdge *);
    void setEdgeOdom();
    void setNodeRotation();
    void saveBuiltMap();
    void loadBuiltMap();
    void initROS();
    void changeNodeMovable(bool);
    void askForRealTimeMap();
    void displayCandidateFromRealTime(int);
};

#endif // TOPOUI_H
