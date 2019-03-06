//
// Created by stumbo on 18-10-17.
//

#ifndef TOPOLOGY_MAP_TOPONODEGVIEW_H
#define TOPOLOGY_MAP_TOPONODEGVIEW_H

#include <QGraphicsView>
#include <QWidget>
#include <QPointF>

#include "include/topo/Topo.h"
#include "QNode.h"

class TopoNodeGView : public QGraphicsView {
    Q_OBJECT

public:
    explicit TopoNodeGView(QWidget *parent = nullptr) : QGraphicsView(parent){}
    ~TopoNodeGView() override;

    NodeInstance *getTheDrawnInstance();

    void startDrawingIns();

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    NodeInstance * drawingIns{};
    QGraphicsLineItem * theDrawingLine{};
    bool drawingLine = false;

    QLineF getLineDrawingFromOrigin(const QPointF point);
};


#endif //TOPOLOGY_MAP_TOPONODEGVIEW_H
