#include "topoui.h"
#include "ui_topoui.h"

#include "topo/Topo.h"

#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QPointF>
#include <QHash>
#include <QDebug>

#include <map>
#include <queue>

using namespace std;

TopoUI::TopoUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TopoUI)
{
    ui->setupUi(this);
    connect(ui->btnInputMap, SIGNAL(clicked()), this, SLOT(loadMapFromFile()));
    connect(ui->cmboMapCandidate, SIGNAL(activated(int)),
            this, SLOT(displayTheActivitedMap(int)));
    ui->mapGView->setScene(&this->mapScene);
    mapScene.addItem(new QGraphicsRectItem(0,0,100,100));
    mapScene.addItem(new QGraphicsRectItem(100,100,100,100));
//    mapScene.addItem(new QGraphicsRectItem(0,500,100,100));
//    mapScene.addItem(new QGraphicsRectItem(500,0,100,100));
    ui->mapGView->setDragMode(QGraphicsView::ScrollHandDrag);
}

TopoUI::~TopoUI()
{
    delete ui;
}

void TopoUI::paintEvent(QPaintEvent *event) {
//    QWidget::paintEvent(event);
//    QPainter painter(this);
//    painter.setWindow(QRect(-50, -50, 500, 500));
//    painter.drawArc(QRectF{10.0, 20.0, 280.0, 260.0}, 30 * 16, 120 * 16);
}

void TopoUI::loadMapFromFile() {
    if (mapGroup.reloadFromFile(ui->etInputMap->text().toStdString())) {
        cout << "UI load map successful" << endl;
        comboBoxMaps.clear();

        for (const auto & mapCand: mapGroup.getMapCollection().getMaps()) {
            static int i = 0;
            QString comboInfo = QString("#%1 fullEdge:%2")
                    .arg(i).arg(mapCand->getFullEdgeNumber());
            comboBoxMaps.push_back(mapCand);
            ui->cmboMapCandidate->addItem(comboInfo);
            i++;
        }

    } else {
        cerr << "UI load map FAIL" << endl;
    }
}

void TopoUI::displayTheActivitedMap(int index) {
    MapCandidate & map2Draw = *comboBoxMaps[index];
    map2Draw.cleanAllNodeFlags();

    auto beginNode = map2Draw.getOneTopoNode();

    QHash<TopoNode *, QPointF> nodePose;
    queue<TopoNode*> lookupQueue;

    nodePose.insert(beginNode, QPointF(0.0, 0.0));
    lookupQueue.push(beginNode);

    while(!lookupQueue.empty()) {
        auto & curNode = lookupQueue.front();
        lookupQueue.pop();
        QPointF curPointF = nodePose.value(curNode);
        for (gateId edgeNo = 0; edgeNo < curNode->getInsCorrespond()->sizeOfExits(); edgeNo++) {

            //means this edge have been drawn
            if (curNode->chkFlag(edgeNo)) {
                continue;
            }

            auto curEdge = curNode->getEdge(edgeNo);
            if (curEdge == nullptr) {
                continue;
            }

            TopoNode * anotherNode = curEdge->getAnotherNode(curNode);

            anotherNode->setFlag(curEdge->getAnotherGate(curNode));

            if (nodePose.find(anotherNode) != nodePose.end()) {
                //TODO draw the edge
                continue;
            }

            lookupQueue.push(anotherNode);
            auto odomData = curEdge->getOdomData(curNode);
            QPointF dist{odomData.first, odomData.second};
            nodePose.insert(anotherNode, curPointF + dist);
        }
    }
    for (auto it = nodePose.begin(); it != nodePose.end(); it++) {
        auto tempIns = it.key()->getInsCorrespond();
        QString tempStr = "(";
        for (const auto exit: tempIns->getExits()) {
            tempStr.append(QString::number(exit.getOutDir()) + ",");
        }
        tempStr.append(")");
        qDebug() << tempStr << ", " << it.value();
    }
//    cout << "it complete" << endl;
}
