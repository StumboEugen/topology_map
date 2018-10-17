#include "topoui.h"
#include "ui_topoui.h"

#include "topo/Topo.h"

#include "QGI_Node.h"
#include "QGI_Edge.h"

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

    mapGView = new TopoMapGView(ui->centralWidget);
    mapGView->setObjectName(QString::fromUtf8("mapGView"));
    mapGView->setGeometry(QRect(10, 10, 601, 401));

    connect(ui->btnInputMap, SIGNAL(clicked()), this, SLOT(loadMapFromFile()));
    connect(ui->cmboMapCandidate, SIGNAL(activated(int)),
            this, SLOT(displayTheActivitedMap(int)));
    mapGView->setScene(&this->mapScene);
//    mapScene.addItem(new QGraphicsRectItem(0,500,100,100));
//    mapScene.addItem(new QGraphicsRectItem(500,0,100,100));

//    mapGView->setDragMode(QGraphicsView::ScrollHandDrag);
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
    mapScene.clear();

//    mapScene.removeItem(mapScene.itemAt({50, 50}));

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
    mapScene.clear();

    MapCandidate & map2Draw = *comboBoxMaps[index];
    map2Draw.cleanAllNodeFlags();

    auto beginNode = map2Draw.getOneTopoNode();

    queue<TopoNode*> lookupQueue;

    auto nodeQGI = new QGI_Node(beginNode);
    beginNode->setAssistPtr(nodeQGI);
    mapScene.addItem(nodeQGI);
    lookupQueue.push(beginNode);

    while(!lookupQueue.empty()) {
        auto & curNode = lookupQueue.front();
        lookupQueue.pop();
        QPointF curPos = static_cast<QGI_Node*>(curNode->getAssistPtr())->pos();

        //look into every edge
        for (gateId edgeNo = 0; edgeNo < curNode->getInsCorrespond()->sizeOfExits(); edgeNo++) {

//            //means this edge have been drawn
//            if (curNode->chkFlag(edgeNo)) {
//                continue;
//            }

            auto curEdge = curNode->getEdge(edgeNo);
            //means this edge has never been moved through
            if (curEdge == nullptr) {
                continue;
            }

            TopoNode * anotherNode = curEdge->getAnotherNode(curNode);
            anotherNode->setFlag(curEdge->getAnotherGate(curNode));

            //means the "anotherNode" has been drawn
            if (anotherNode->getAssistPtr() != nullptr) {
                auto edge2Draw = new QGI_Edge(curEdge
                        , static_cast<QGI_Node *>(curNode->getAssistPtr())
                        , static_cast<QGI_Node *>(anotherNode->getAssistPtr()));
                mapScene.addItem(edge2Draw);
                continue;
            }

            //draw another node
            auto odomData = curEdge->getOdomData(curNode);
            QPointF dist{odomData.first, odomData.second};
            nodeQGI = new QGI_Node(anotherNode);
            anotherNode->setAssistPtr(nodeQGI);
            nodeQGI->setPos(curPos + dist * METER_TO_PIXLE);
            mapScene.addItem(nodeQGI);

            lookupQueue.push(anotherNode);
        }
    }

    for (const auto & node: map2Draw.getNodes()) {
        auto tempIns = node->getInsCorrespond();
        QString tempStr = "(";
        for (const auto exit: tempIns->getExits()) {
            tempStr.append(QString::number(exit.getOutDir()) + ",");
        }
        tempStr.append(")");
        qDebug() << tempStr << ", " << static_cast<QGI_Node*>(node->getAssistPtr())->pos();
    }

//    cout << "it complete" << endl;
}
