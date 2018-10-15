//
// Created by stumbo on 18-10-15.
//

#include "TopoMapGView.h"

#define SCALE_TIME 1.2
#define SCALE_MAX 4.0

void TopoMapGView::wheelEvent(QWheelEvent *event) {
    if (event->modifiers() & Qt::ControlModifier) {
        auto curTrans = transform();
        auto curScale = curTrans.m11();
        if (event->delta() > 0 && curScale <= SCALE_MAX) {
            curTrans.scale(SCALE_TIME, SCALE_TIME);
        } 
        else if (event->delta() < 0 && curScale >= 1 / SCALE_MAX) {
            curTrans.scale(1 / SCALE_TIME, 1 / SCALE_TIME);
        }
        setMatrix(curTrans.toAffine());
    } else {
        QGraphicsView::wheelEvent(event);
    }
}
