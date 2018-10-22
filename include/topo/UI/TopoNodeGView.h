//
// Created by stumbo on 18-10-17.
//

#ifndef TOPOLOGY_MAP_TOPONODEGVIEW_H
#define TOPOLOGY_MAP_TOPONODEGVIEW_H

#include <QGraphicsView>
#include <QWidget>

#include "include/topo/Topo.h"
#include "QGI_Node.h"

class TopoNodeGView : public QGraphicsView {
    Q_OBJECT

public:
    explicit TopoNodeGView(QWidget *parent = nullptr) : QGraphicsView(parent){}
    ~TopoNodeGView() override = default;

    void setDrawingMode(bool drawingMode) {
        TopoNodeGView::drawingMode = drawingMode;
    }

private:
    bool drawingMode = false;
};


#endif //TOPOLOGY_MAP_TOPONODEGVIEW_H
