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

namespace Ui {
    class TopoUI;
    class DockReadMapUI;
    class DockBuildMapUI;
    class DockSimulationUI;
}

class TopoUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit TopoUI(QWidget *parent = nullptr);
    ~TopoUI() override;

    void appendMsg(const QString & msg);
    void setMsg(const QString & msg);

private:

    Ui::TopoUI *uiMain;
    QActionGroup * modeGroup;
    QAction * mode_READ;
    QAction * mode_BUILD;
    QAction * mode_SIMULATION;

    QRobot * robot;

    ros::Publisher pub_nodeInfo;
    ros::Publisher pub_gateMove;

    bool checkROS();

    MapArranger mapFromReading;
    MapArranger mapFromBuilding;

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

    void cleanTableView();
    void cleanReadDock();

    QDockWidget * initTheDock(const char *objectName);

    void displayMapAtMapGV(MapCandidate &, bool detailed = false, bool movable = false);

    bool loadMapGroupFromFile(const QString & fileName, MapArranger & dist);

    void sendNodeROSmsg(QNode *clickedNode, const QEdge *edgeWithRobot, int exit);

private Q_SLOTS:
    void loadReadingMap();
    void displayTheActivitedMap(int);
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
};

#endif // TOPOUI_H
