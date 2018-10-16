//
// Created by stumbo on 18-10-16.
//

#ifndef TOPOLOGY_MAP_QGI_EDGE_H
#define TOPOLOGY_MAP_QGI_EDGE_H

#include <QGraphicsItem>
#include "topo/Topo.h"

class QGI_Node;

class QGI_Edge : public QGraphicsItem {

public:
    QGI_Edge(TopoEdge *edge, QGI_Node *QNodeA, QGI_Node *QNodeB);

private:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

private:
    QRectF boundingRect() const override;

    TopoEdge *relatedEdgeTOPO;
    QGI_Node *QNodeA;
    QGI_Node *QNodeB;
};


#endif //TOPOLOGY_MAP_QGI_EDGE_H
