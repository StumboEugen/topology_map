//
// Created by stumbo on 18-10-15.
//

#ifndef TOPOLOGY_MAP_TOPOMAPGVIEW_H
#define TOPOLOGY_MAP_TOPOMAPGVIEW_H

#include <QGraphicsView>
#include <QWidget>

#include "include/topo/Topo.h"
#include "QGI_Node.h"

class QGI_Edge;

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
    QGI_Edge * theDrawingEdge;

    void setNodesMoveable(bool moveable);

Q_SIGNALS:
    void QGI_Node_clicked(QGI_Node *);
    void newEdgeConnected(TopoEdge *newEdge);

public Q_SLOTS:
    void switch2DrawEdgeMode(bool start);
};


#endif //TOPOLOGY_MAP_TOPOMAPGVIEW_H
