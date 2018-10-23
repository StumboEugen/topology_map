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

#include <vector>
#include <iostream>

namespace Ui {
    class TopoUI;
    class DockReadMapUI;
    class DockBuildMapUI;
}

class TopoUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit TopoUI(QWidget *parent = nullptr);
    ~TopoUI() override;

private:

    Ui::TopoUI *uiMain;
    QActionGroup * modeGroup;
    QAction * mode_READ;
    QAction * mode_BUILD;
    QAction * mode_SIMULATION;

    /**
     * in different mode, it has differen't function
     */
    MapArranger mapGroup;

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

    void cleanEveryThing();

    QDockWidget * initTheDock(const char *objectName);

private Q_SLOTS:
    void loadMapFromFile();
    void displayTheActivitedMap(int);
    void drawTopoNode(TopoNode *);
    void changeMode(QAction *);
    void setMapGViewDragMode(bool);
    void buildModeNewNode();
    void buildModeAddNode2MapView();
};

#endif // TOPOUI_H
