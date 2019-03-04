//
// Created by stumbo on 18-10-16.
//

#ifndef TOPOLOGY_MAP_QGI_EDGE_H
#define TOPOLOGY_MAP_QGI_EDGE_H

#include <QGraphicsItem>
#include "topo/Topo.h"

class QGI_Node;

class QGI_Edge : public QGraphicsLineItem {

public:

    /**
     * constructor for reading mode
     */
    QGI_Edge(TopoEdge *edge, QGI_Node *QNodeA, QGI_Node *QNodeB);

    /**
     * constructor for building mode
     */
    QGI_Edge(QGI_Node *QNode, uint8_t gate);

    void setRelatedEdgeTOPO(TopoEdge *relatedEdgeTOPO);

    void setNodeA(QGI_Node *QNodeA, uint8_t gateA);

    void setNodeB(QGI_Node *QNodeB, uint8_t gateB);

    QPainterPath shape() const override ;

    void setLength(double length);

protected:
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) override;

    void focusInEvent(QFocusEvent *event) override;

    void focusOutEvent(QFocusEvent *event) override;

private:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;

    QRectF boundingRect() const override;

    TopoEdge *relatedEdgeTOPO = nullptr;
    QGI_Node *QNodeA = nullptr;
    QGI_Node *QNodeB = nullptr;
    uint8_t gateA;
    uint8_t gateB;

    double odomX = 0.0;
    double odomY = 0.0;
    double length = 0.0;

};


#endif //TOPOLOGY_MAP_QGI_EDGE_H
