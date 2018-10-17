#ifndef TOPOUI_H
#define TOPOUI_H

#define METER_TO_PIXLE 30

#include <QMainWindow>
#include <QGraphicsScene>
#include "topo/Topo.h"

#include "TopoMapGView.h"
#include "TopoNodeGView.h"

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
    MapArranger mapGroup;
    Ui::TopoUI *ui;

    TopoMapGView * mapGView;
    QGraphicsScene mapScene;

    TopoNodeGView * nodeGView;
    QGraphicsScene nodeScene;

    std::vector<MapCandidate*> comboBoxMaps;
    void paintEvent(QPaintEvent * event) override;

private Q_SLOTS:
    void loadMapFromFile();
    void displayTheActivitedMap(int);
    void drawTopoNode(TopoNode *);
};

#endif // TOPOUI_H
