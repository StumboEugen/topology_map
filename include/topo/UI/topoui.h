#ifndef TOPOUI_H
#define TOPOUI_H

#include <QMainWindow>
#include <QGraphicsScene>
#include "topo/Topo.h"

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
    QGraphicsScene mapScene;
    void paintEvent(QPaintEvent * event) override;

private Q_SLOTS:
    void loadMapFromFile();
};

#endif // TOPOUI_H
