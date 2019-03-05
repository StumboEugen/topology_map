//
// Created by stumbo on 19-3-5.
//

#ifndef TOPOLOGY_MAP_QGI_ROBOT_H
#define TOPOLOGY_MAP_QGI_ROBOT_H


#include <QGraphicsItem>

class QGI_Robot : public QGraphicsItem  {

public:
    explicit QGI_Robot();

    void
    paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    QRectF boundingRect() const override;

    QPainterPath shape() const override;

    ~QGI_Robot(){};

private:

};


#endif //TOPOLOGY_MAP_QGI_ROBOT_H
