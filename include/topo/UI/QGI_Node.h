//
// Created by stumbo on 18-10-16.
//

#ifndef TOPOLOGY_MAP_QGI_NODE_H
#define TOPOLOGY_MAP_QGI_NODE_H

#define RECT_SIZE_HALF 35
#define QGI_NODE_TYPE 2000

#include <QGraphicsItem>
#include "topo/Topo.h"

class QGI_Node : public QGraphicsItem {
protected:

public:
    explicit QGI_Node(TopoNode * node);

    QRectF boundingRect() const override;

    QPainterPath shape() const override;

    int whichExitIsAtPos(const QPointF &);
    QPointF posOfExitInItem(int index);
    QPointF posOfExitInScene(int index);


public:

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;

    TopoNode *getRelatedNodeTOPO() const {
        return relatedNodeTOPO;
    }

    void setDrawDetail(bool drawDetail) {
        QGI_Node::drawDetail = drawDetail;
    }

    int type() const override;

private:
    TopoNode * relatedNodeTOPO;
    bool drawDetail = false;

    static QRect outline() {
        return {-RECT_SIZE_HALF - 1, -RECT_SIZE_HALF - 1,
                RECT_SIZE_HALF * 2 + 2, RECT_SIZE_HALF * 2 + 2};
    }
};


#endif //TOPOLOGY_MAP_QGI_NODE_H
