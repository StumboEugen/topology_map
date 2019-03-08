//
// Created by stumbo on 18-10-16.
//

#ifndef TOPOLOGY_MAP_QGI_EDGE_H
#define TOPOLOGY_MAP_QGI_EDGE_H

#include <QGraphicsItem>
#include "topo/Topo.h"

class QNode;

class QEdge : public QGraphicsLineItem {

public:

    /**
     * constructor for reading mode
     */
    QEdge(TopoEdge *edge, QNode *QNodeA, QNode *QNodeB);

    /**
     * constructor for building mode
     */
    QEdge(QNode *QNode, uint8_t gate);

    void setRelatedEdgeTOPO(TopoEdge *relatedEdgeTOPO);

    void setNodeA(QNode *QNodeA, uint8_t gateA);

    void setNodeB(QNode *QNodeB, uint8_t gateB);

    QNode * getAnotherNode(QNode *);

    QPainterPath shape() const override ;

    void setLength(double length);

    void refreshOdom();

    TopoEdge *getRelatedEdgeTOPO() const;

protected:
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) override;

    void focusInEvent(QFocusEvent *event) override;

    void focusOutEvent(QFocusEvent *event) override;

private:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;

    QRectF boundingRect() const override;

    void registerAtBothNodes();

    void showMsg();

private:
    TopoEdge *relatedEdgeTOPO = nullptr;
    QNode *QNodeA = nullptr;
    QNode *QNodeB = nullptr;
    uint8_t gateA;
    uint8_t gateB;

    double length = 0.0;

};


#endif //TOPOLOGY_MAP_QGI_EDGE_H
