//
// Created by stumbo on 18-10-16.
//

#ifndef TOPOLOGY_MAP_QGI_NODE_H
#define TOPOLOGY_MAP_QGI_NODE_H

#define RECT_SIZE_HALF 30
#define QGI_NODE_TYPE 2000

#include <QGraphicsItem>

#include <vector>

#include "topo/Topo.h"

class QEdge;

class QNode : public QGraphicsItem  {

public:
    explicit QNode(TopoNode * node);

    QRectF boundingRect() const override;

    QPainterPath shape() const override;

    int whichExitIsAtPos(const QPointF &);
    QPointF posOfExitInItem(int index);
    QPointF posOfExitInScene(int index);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;

    TopoNode *getRelatedNodeTOPO() const {
        return relatedNodeTOPO;
    }

    void setDrawDetail(bool drawDetail) {
        QNode::drawDetail = drawDetail;
    }

    void setRealTimeMode(bool realTimeMode) {
        QNode::realTimeMode = realTimeMode;
        setAcceptHoverEvents(realTimeMode);
    }

    int type() const override;

    void setQEdge(int index, QEdge * edge);

    QEdge * getQEdgeAtExit(int index) {
        return connectedEdges[index];
    }

    QNode * getQNodeAtExit(int index);

    int getExitNums();

    /// in degree, clock wise
    void setRotation(double angle);

protected:
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) override;

    void hoverMoveEvent(QGraphicsSceneHoverEvent *event) override;

    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;

    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;

private:
    TopoNode * relatedNodeTOPO;
    bool drawDetail = false;

    bool realTimeMode = false;
    int currentExitHoverOn = -1;

    /// in degree clock wise
    double rotation = 0.0;

    static QRect outline() {
        return {-RECT_SIZE_HALF - 1, -RECT_SIZE_HALF - 1,
                RECT_SIZE_HALF * 2 + 2, RECT_SIZE_HALF * 2 + 2};
    }

    std::vector<QEdge*> connectedEdges;
};


#endif //TOPOLOGY_MAP_QGI_NODE_H
