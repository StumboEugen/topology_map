//
// Created by stumbo on 18-10-15.
//

#ifndef TOPOLOGY_MAP_TOPOMAPGVIEW_H
#define TOPOLOGY_MAP_TOPOMAPGVIEW_H

#include <QGraphicsView>
#include <QWidget>

#include "include/topo/Topo.h"
#include "QGI_Node.h"

class TopoMapGView : public QGraphicsView{
    Q_OBJECT

public:
    explicit TopoMapGView(QWidget *parent = nullptr) : QGraphicsView(parent){}
    ~TopoMapGView() override = default;


protected:
    void wheelEvent(QWheelEvent * event) override ;
    void mousePressEvent(QMouseEvent * event) override ;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    bool drawingEdgeMode = false;
    bool isDrawingEdge = false;
    uint8_t exitAofDrawingEdge;
    TopoNode * nodeAofDrawingEdge;
    QGraphicsLineItem * theDrawingEdge;

    void setNodesMoveable(bool moveable);

Q_SIGNALS:
    void QGI_Node_clicked(TopoNode *);
    void newEdgeConnected(TopoNode *, uint8_t, TopoNode *, uint8_t);

public Q_SLOTS:
    void drawingEdge(bool start);
};


#endif //TOPOLOGY_MAP_TOPOMAPGVIEW_H
