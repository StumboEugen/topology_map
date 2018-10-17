//
// Created by stumbo on 18-10-15.
//

#ifndef TOPOLOGY_MAP_TOPOMAPGVIEW_H
#define TOPOLOGY_MAP_TOPOMAPGVIEW_H

#include <QGraphicsView>
#include <QWidget>

class TopoMapGView : public QGraphicsView{
    Q_OBJECT

public:
    explicit TopoMapGView(QWidget *parent = nullptr) : QGraphicsView(parent){}
    ~TopoMapGView() override = default;

protected:
    void wheelEvent(QWheelEvent * event) override ;
    void mousePressEvent(QMouseEvent * event) override ;
};


#endif //TOPOLOGY_MAP_TOPOMAPGVIEW_H
