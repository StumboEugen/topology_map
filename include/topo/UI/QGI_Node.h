//
// Created by stumbo on 18-10-16.
//

#ifndef TOPOLOGY_MAP_QGI_NODE_H
#define TOPOLOGY_MAP_QGI_NODE_H

#define RECT_SIZE_HALF 30

#include <QGraphicsItem>
#include "topo/Topo.h"

class QGI_Node : public QGraphicsItem {
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;

public:
    explicit QGI_Node(TopoNode * node);

    QRectF boundingRect() const override;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;

    TopoNode *getRelatedNodeTOPO() const {
        return relatedNodeTOPO;
    }

private:
    TopoNode * relatedNodeTOPO;
};


#endif //TOPOLOGY_MAP_QGI_NODE_H
