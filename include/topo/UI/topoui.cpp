#include "topoui.h"
#include "ui_topoui.h"

#include "topo/Topo.h"

#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsRectItem>

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
    cout << comboBoxMaps[index]->getFullEdgeNumber() << endl;
}
