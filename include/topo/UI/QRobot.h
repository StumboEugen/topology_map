//
// Created by stumbo on 19-3-5.
//

#ifndef TOPOLOGY_MAP_QGI_ROBOT_H
#define TOPOLOGY_MAP_QGI_ROBOT_H


#include <QGraphicsItem>

class QNode;
class QEdge;

class QRobot : public QGraphicsItem  {

public:
    explicit QRobot(QGraphicsItem * currentAt = nullptr);

    void
    paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    QRectF boundingRect() const override;

    QPainterPath shape() const override;

    void move2(QGraphicsItem * target);

    QGraphicsItem *getCurrentAt() const;

private:

    QGraphicsItem * currentAt;

};


#endif //TOPOLOGY_MAP_QGI_ROBOT_H
