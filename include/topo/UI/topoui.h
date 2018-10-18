#ifndef TOPOUI_H
#define TOPOUI_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QActionGroup>

#include "topo/Topo.h"
#include "TopoMapGView.h"
#include "TopoNodeGView.h"
#include "UITOOLS.h"

#include <vector>
#include <iostream>

namespace Ui {
class TopoUI;
}

class TopoUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit TopoUI(QWidget *parent = nullptr);
    ~TopoUI() override;

private:
    QActionGroup * workingModeGroup;
    QAction * mode_READ;
    QAction * mode_BUILD;
    QAction * mode_SIMULATION;

    MapArranger mapGroup;
    Ui::TopoUI *ui;

    QHBoxLayout * centerLayout;

    TopoMapGView * mapGView;
    QGraphicsScene mapScene;

    QVBoxLayout * smallWindowLayout;

    TopoNodeGView * nodeGView;
    QGraphicsScene nodeScene;

    std::vector<MapCandidate*> comboBoxMaps;

    void cleanEveryThing();

private Q_SLOTS:
    void loadMapFromFile();
    void displayTheActivitedMap(int);
    void drawTopoNode(TopoNode *);
};

#endif // TOPOUI_H
